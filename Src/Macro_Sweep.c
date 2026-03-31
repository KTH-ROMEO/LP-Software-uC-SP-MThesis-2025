/*
 * Macro_Sweep.c
 *
 * Unified macro sweep transaction manager for PUS8 function 0xD1
 */

#include "Macro_Sweep.h"
#include "PUS_8_service.h"
#include "General_Functions.h"
#include "Space_Packet_Protocol.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "queue.h"

/* Externals */
extern QueueHandle_t UART_OBC_Out_Queue;
extern UART_HandleTypeDef huart5;

extern uint8_t UART_FPGA_Rx_Buffer[100];
extern uint8_t UART_FPGA_OBC_Tx_Buffer[100];

/* Runtime context for one active macro sweep transaction */
typedef struct {
    bool active;                             
    uint8_t requested_subop;                
    MacroSweep_State_t state;       // transaction state

    bool need_metadata;                      
    bool need_table;                        
    bool request_full_table;        // false = n_steps table, true = full table 

    bool received_meta1;                          
    bool received_meta2;                      
    uint8_t meta1[5];                      
    uint8_t meta2[6];                        

    bool end_marker_seen;                    
    uint16_t expected_steps;                // number of table rows expected 
    uint16_t received_step_count;           // number of unique step rows stored 
    uint16_t tab0[SWEEP_TABLE_MAX_STEPS];    
    uint16_t tab1[SWEEP_TABLE_MAX_STEPS];   
    uint8_t step_received[SWEEP_TABLE_MAX_STEPS];   // 1 if step already received 
} Macro_Context_t;

static Macro_Context_t ctx;

/* Private helpers */

// Sends one macro request to FPGA 
static TM_Err_Codes MacroSweep_SendFpgaRequest(uint8_t cmd_id, uint8_t arg, bool has_arg) {
    uint8_t msg[5];
    uint8_t msg_cnt = 0;

    msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
    msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
    msg[msg_cnt++] = cmd_id;

    if (has_arg)
        msg[msg_cnt++] = arg;

    msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

    memset(UART_FPGA_Rx_Buffer, 0, sizeof(UART_FPGA_Rx_Buffer));
    memset(UART_FPGA_OBC_Tx_Buffer, 0, sizeof(UART_FPGA_OBC_Tx_Buffer));

    UART_FPGA_OBC_Tx_Buffer[0] = cmd_id;
    if (has_arg)
        UART_FPGA_OBC_Tx_Buffer[1] = arg;

    if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOB, LED4_Pin | LED3_Pin, GPIO_PIN_SET);
        return DEV_CPDU_EXEC_FAIL;
    }

    return NO_ERROR;
}

// Builds one TM packet with the macro sweep miniheader and queues it for OBC TX 
static bool MacroSweep_QueueTMPacket(uint8_t subop, MacroSweep_TmPacketType_t pkt_type,
                                     uint8_t total_steps, uint8_t start_step_index,
                                     const uint8_t *payload, uint16_t payload_len) {
    UART_OUT_OBC_msg msg = {0};
    MacroSweep_TM_MiniHeader_t header;

    msg.PUS_HEADER_PRESENT = 0;

    header.function_id = MACRO_SWEEP_BIAS_CONFIG;
    header.subop = subop;
    header.packet_type = (uint8_t)pkt_type;
    header.total_steps = total_steps;           // last valid step index 
    header.start_step_index = start_step_index; // first step in TM

    memcpy(&msg.TM_data[0], &header, sizeof(header));

    if ((payload != NULL) && (payload_len > 0))
        memcpy(&msg.TM_data[sizeof(header)], payload, payload_len);

    msg.TM_data_len = (uint16_t)(sizeof(header) + payload_len);

    return (xQueueSend(UART_OBC_Out_Queue, &msg, portMAX_DELAY) == pdPASS);
}

// Sends buffered metadata in one TM  
static bool MacroSweep_SendMetadataTM(void) {
    uint8_t payload[11];

    if (!ctx.received_meta1 || !ctx.received_meta2)
        return false;

    memcpy(payload, ctx.meta1, sizeof(ctx.meta1));
    memcpy(&payload[sizeof(ctx.meta1)], ctx.meta2, sizeof(ctx.meta2));

    return MacroSweep_QueueTMPacket(ctx.requested_subop, MACRO_TM_TYPE_METADATA,
                                    0, 0,
                                    payload, sizeof(payload));
}

// Sends buffered table rows in chunked TMs
static bool MacroSweep_SendTableTMs(void) {
    uint16_t start_step = 0;
    uint16_t steps_2_send = ctx.expected_steps;
    MacroSweep_TmPacketType_t pkt_type;

    if ((steps_2_send == 0) || (steps_2_send > SWEEP_TABLE_MAX_STEPS))
        return false;

    pkt_type = ctx.request_full_table ? MACRO_TM_TYPE_TABLE_FULL : MACRO_TM_TYPE_TABLE_NSTEP;  // 
    uint8_t last_received_step_index = (uint8_t)(steps_2_send - 1);    // total steps in miniheader

    while (start_step < steps_2_send) {
        uint8_t payload[MACRO_TM_ENTRIES_PER_PKT * MACRO_TM_TABLE_ROW_LEN];
        uint16_t payload_index = 0;
        uint16_t end_step = start_step + MACRO_TM_ENTRIES_PER_PKT;

        if (end_step > steps_2_send)
            end_step = steps_2_send;

        for (uint16_t step = start_step; step < end_step; step++) {
            if (ctx.step_received[step] == 0)  // require every step from FPGA table
                return false;

            payload[payload_index++] = (uint8_t)((ctx.tab0[step] >> 8) & 0xFF);
            payload[payload_index++] = (uint8_t)(ctx.tab0[step] & 0xFF);
            payload[payload_index++] = (uint8_t)((ctx.tab1[step] >> 8) & 0xFF);
            payload[payload_index++] = (uint8_t)(ctx.tab1[step] & 0xFF);
        }

        if (!MacroSweep_QueueTMPacket(ctx.requested_subop, pkt_type,
                                      last_received_step_index, (uint8_t)start_step,
                                      payload, payload_index)) {
            return false;
        }
        start_step = end_step;
    }
    return true;
}

// Advances transaction only when current phase is complete
static TM_Err_Codes MacroSweep_AdvanceTransaction(void) {
    if ((ctx.state == MACRO_STATE_WAIT_METADATA) && ctx.received_meta1 && ctx.received_meta2) {
        if (ctx.need_table) {
            ctx.state = MACRO_STATE_WAIT_TABLE;

            if (MacroSweep_SendFpgaRequest(MACRO_GET_SB_TABLEs, ctx.request_full_table ? (uint8_t)MACRO_FPGA_TABLE_FULL : (uint8_t)MACRO_FPGA_TABLE_NSTEPS, true) != NO_ERROR) 
                return DEV_CPDU_EXEC_FAIL;

            return NO_ERROR;
        }

        ctx.state = MACRO_STATE_SENDING_TM;
        if (!MacroSweep_SendMetadataTM())
            return DEV_CPDU_EXEC_FAIL;

        ctx.state = MACRO_STATE_DONE;
        MacroSweep_Reset();
        return NO_ERROR;
    }

    if ((ctx.state == MACRO_STATE_WAIT_TABLE) && ctx.end_marker_seen && (ctx.expected_steps != 0) &&
        (ctx.expected_steps <= SWEEP_TABLE_MAX_STEPS) && (ctx.received_step_count == ctx.expected_steps)) {
        ctx.state = MACRO_STATE_SENDING_TM;

        if (ctx.need_metadata && !MacroSweep_SendMetadataTM())
            return DEV_CPDU_EXEC_FAIL;

        if (!MacroSweep_SendTableTMs())
            return DEV_CPDU_EXEC_FAIL;

        ctx.state = MACRO_STATE_DONE;
        MacroSweep_Reset();
        return NO_ERROR;
    }

    return BAD_STATE;
}


/* Public API */
void MacroSweep_Reset(void) {
    memset(&ctx, 0, sizeof(ctx));
    ctx.state = MACRO_STATE_IDLE;
}

// Returns true while a macro sweep transaction is active
bool MacroSweep_Active(void) {
    return ctx.active;
}

// Start macro sweep transaction
TM_Err_Codes MacroSweep_StartTransaction(uint8_t subop) {
    uint8_t cmd_id = 0;
    uint8_t arg = 0;
    bool has_arg = false;

    MacroSweep_Reset();
    ctx.active = true;
    ctx.requested_subop = subop;

    switch (subop) {
        case 0x01:
            ctx.need_metadata = true;
            ctx.need_table = false;
            ctx.request_full_table = false;
            ctx.state = MACRO_STATE_WAIT_METADATA;

            cmd_id = MACRO_GET_SB_METADATA;
            arg = 0;
            has_arg = false;
            break;

        case 0x02:
            ctx.need_metadata = false;
            ctx.need_table = true;
            ctx.request_full_table = false;
            ctx.expected_steps = 0; // determined later from end marker step 
            ctx.state = MACRO_STATE_WAIT_TABLE;

            cmd_id = MACRO_GET_SB_TABLEs;
            arg = (uint8_t)MACRO_FPGA_TABLE_NSTEPS;
            has_arg = true;
            break;

        case 0x03:
            ctx.need_metadata = false;
            ctx.need_table = true;
            ctx.request_full_table = true;
            ctx.expected_steps = SWEEP_TABLE_MAX_STEPS;
            ctx.state = MACRO_STATE_WAIT_TABLE;

            cmd_id = MACRO_GET_SB_TABLEs;
            arg = (uint8_t)MACRO_FPGA_TABLE_FULL;
            has_arg = true;
            break;

        case 0x04:
            ctx.need_metadata = true;
            ctx.need_table = true;
            ctx.request_full_table = false;
            ctx.expected_steps = 0;
            ctx.state = MACRO_STATE_WAIT_METADATA;

            cmd_id = MACRO_GET_SB_METADATA;
            arg = 0;
            has_arg = false;
            break;

        case 0x05:
            ctx.need_metadata = true;
            ctx.need_table = true;
            ctx.request_full_table = true;
            ctx.expected_steps = SWEEP_TABLE_MAX_STEPS;
            ctx.state = MACRO_STATE_WAIT_METADATA;

            cmd_id = MACRO_GET_SB_METADATA;
            arg = 0;
            has_arg = false;
            break;

        default:
            MacroSweep_Reset();
            return UNDEFINED_PARAM_ID;
    }

    if (MacroSweep_SendFpgaRequest(cmd_id, arg, has_arg) != NO_ERROR) {
        MacroSweep_Reset();
        return DEV_CPDU_EXEC_FAIL;
    }

    return NO_ERROR;
}

// Handles FPGA reply frame for the active macro sweep transaction
bool MacroSweep_HandleFpgaFrame(const uint8_t *frame) {
    if (!ctx.active || (frame == NULL))
        return false;

    if ((frame[0] != FPGA_MSG_PREAMBLE_0) || (frame[1] != FPGA_MSG_PREAMBLE_1) || (frame[FPGA_FRAME_LEN - 1U] != FPGA_MSG_POSTAMBLE))
        return false;

    switch (ctx.state) {
        case MACRO_STATE_WAIT_METADATA:
            const uint8_t *payload = &frame[3];

            if (frame[2] != MACRO_GET_SB_METADATA)
                return false;

            if (!ctx.received_meta1) {
                memcpy(ctx.meta1, payload, sizeof(ctx.meta1));
                ctx.received_meta1 = true;
                return true;
            }

            if (!ctx.received_meta2) {
                memcpy(ctx.meta2, payload, sizeof(ctx.meta2));
                ctx.received_meta2 = true;
            }

            if (ctx.received_meta1 && ctx.received_meta2) {
                if (MacroSweep_AdvanceTransaction() != NO_ERROR)
                    MacroSweep_Reset();
            }

            return true;

        case MACRO_STATE_WAIT_TABLE:
            if (frame[2] != MACRO_GET_SB_TABLEs)
                return false;

            // Decode table row: frame[3][4:5][6:7] = [step index][tab0 value][tab1 value] 
            uint8_t step = frame[3];
            if (step >= SWEEP_TABLE_MAX_STEPS)
                return true;

            uint16_t t0 = ((uint16_t)frame[4] << 8) | (uint16_t)frame[5];
            uint16_t t1 = ((uint16_t)frame[6] << 8) | (uint16_t)frame[7];

            // Count a step once 
            if (ctx.step_received[step] == 0) {
                ctx.received_step_count++;
                ctx.step_received[step] = 1;
            }

            ctx.tab0[step] = t0;
            ctx.tab1[step] = t1;

            // End-of-table marker: frame[8:10] = 0xEE 0xEE 0xEE
            if ((frame[8] == MACRO_FPGA_TABLE_END_MARK) && (frame[9] == MACRO_FPGA_TABLE_END_MARK) && (frame[10] == MACRO_FPGA_TABLE_END_MARK)) {
                ctx.end_marker_seen = true;

                if (!ctx.request_full_table)
                    ctx.expected_steps = (uint16_t)step + 1;    // n-step mode: expected_steps = last_received_step + 1
            }

            if (ctx.end_marker_seen && (ctx.expected_steps != 0) &&
                (ctx.expected_steps <= SWEEP_TABLE_MAX_STEPS) && (ctx.received_step_count == ctx.expected_steps)) {
                if (MacroSweep_AdvanceTransaction() != NO_ERROR)
                    MacroSweep_Reset();
            }

            return true;

        default:
            return false;
    }
}
