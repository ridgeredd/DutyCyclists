#ifndef FSM_H
#define FSM_H

#include <stdint.h>

// Log that ptt found an edge
void FSM_flag_ptt_edge();

// Log that transmission should be reallowed
void FSM_flag_tx_reenable();

// Log that transmission has finished
void FSM_flag_tx_finished();

// Log that automatic transmission timer has pinged
void FSM_flag_auto_tx();

// Reset all flags
void FSM_flag_reset_all();

// Call this inside of main while loop
void fsm_main();

// Initialize
void fsm_init();

// set ptt value
void FSM_set_ptt( uint8_t PTT_new_level );

#endif