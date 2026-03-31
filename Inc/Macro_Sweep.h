/*
 * Macro_Sweep.h
 *
 * Existing sweep configuration transfer from FPGA (PUS8 Function ID 0xD1)
*/

#ifndef MACRO_SWEEP_H_
#define MACRO_SWEEP_H_

#include "General_Functions.h"

#include <stdint.h>
#include <stdbool.h>

#define MACRO_TM_MINIHEADER_LEN     5       // TM MiniHeader length 
#define MACRO_TM_TABLE_ROW_LEN      4       // table row length per step
#define MACRO_TM_ENTRIES_PER_PKT    55      // table steps in a TM 
#define MACRO_FPGA_TABLE_END_MARK   0xEE    // only in table end packets
#define SWEEP_TABLE_MAX_STEPS       256     // max sweep table steps

typedef enum {   
    MACRO_GET_SB_METADATA       = 0xD0, 
    MACRO_GET_SB_TABLEs         = 0xD9
} MACRO_FpgaCmd_ID;  // MCU <-> FPGA UART command IDs

typedef enum{
    MACRO_FPGA_TABLE_NSTEPS     = 0x01,
    MACRO_FPGA_TABLE_FULL       = 0x02
} MACRO_FpgaTableMode_t; // FPGA table req mode

typedef enum {
    MACRO_STATE_IDLE = 0,
    MACRO_STATE_WAIT_METADATA,
    MACRO_STATE_WAIT_TABLE,
    MACRO_STATE_SENDING_TM,
    MACRO_STATE_DONE
} MacroSweep_State_t;

typedef enum{
    MACRO_TM_TYPE_METADATA      = 0x00,
    MACRO_TM_TYPE_TABLE_NSTEP   = 0x01,
    MACRO_TM_TYPE_TABLE_FULL    = 0x02
} MacroSweep_TmPacketType_t; 

typedef struct __attribute__((__packed__)) {
    uint8_t function_id;
    uint8_t subop;
    uint8_t packet_type;
    uint8_t total_steps;        // same as table steps [0...255]
    uint8_t start_step_index;   // frist step carried in this TM
} MacroSweep_TM_MiniHeader_t;

/* Public API */
TM_Err_Codes MacroSweep_StartTransaction(uint8_t subop); // Start macro sweep transactions based on subop
void MacroSweep_Reset(void);    // Reset state machine & clear buffered data
bool MacroSweep_Active(void);   // Return true when macro transaction active
bool MacroSweep_HandleFpgaFrame(const uint8_t *frame);   // Processe macro frames and update transaction state

#endif /* MACRO_SWEEP_H_ */
