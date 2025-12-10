#include "tx.h"
#include "fsm.h"
#include "auto_tx.h"
#include "ptt.h"
#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"

// TODO: use key_ptt() somewhere

//=================================== GLOBALS ===============================================

typedef enum {
    //FSM_START = 0,   // Start
    FSM_IDLE = 0,    // Nothing pressed
    FSM_VOICE,       // PTT pressed
    FSM_PTT_KEY,      // Key PTT
    FSM_START_TX,    // PTT released but yet to initiate gps tx
    FSM_TX,          // Currently transmitting gps
    FSM_NUM_STATES
} FSM_State;

static struct {
    unsigned PTT_edge        : 1;  // external "GO" (e.g., button or command)
    unsigned TX_reenable     : 1;  // transmission is reenabled
    unsigned TX_finished     : 1;  // done transmitting
    unsigned auto_TX         : 1;  // automatically transmit
} FSM_flags;

static FSM_State cur_state;

static uint8_t PTT_level;
static uint8_t PTT_prev_level;
static uint8_t TX_enabled;

// helper functions to determine if were pressed / released from flags
static inline uint8_t was_ptt_pressed() { return !PTT_level && PTT_prev_level; }
static inline uint8_t was_ptt_released() { return PTT_level && !PTT_prev_level; }
static inline void update_ptt() { PTT_prev_level = PTT_level; PTT_level = get_ptt(); }

// Static declarations
static FSM_State fsm_idle();
static FSM_State fsm_voice();
static FSM_State fsm_ptt_key();
static FSM_State fsm_start_tx();
static FSM_State fsm_tx();

//======================================== API ===========================================================

// TODO: have this take in the new edge value and combine with ptt_level
void FSM_flag_ptt_edge() { FSM_flags.PTT_edge = 1; }

void FSM_flag_tx_reenable() { FSM_flags.TX_reenable = 1; }

void FSM_flag_tx_finished() { FSM_flags.TX_finished = 1; }

void FSM_flag_auto_tx() { FSM_flags.auto_TX = 1; }

void FSM_flag_reset_all() {

    FSM_flags.TX_reenable  = 0;
    FSM_flags.PTT_edge     = 0;
    FSM_flags.TX_finished  = 0;
    FSM_flags.auto_TX      = 0;
}

// TODO: unneeded?
// Helpful still, but scuffed
void FSM_set_ptt( uint8_t PTT_new_level ) {

    PTT_prev_level = PTT_level;
    PTT_level = PTT_new_level;
    FSM_flags.PTT_edge = PTT_level ^ PTT_prev_level;

}

// call this inside of main while loop
void fsm_main() {

    // TODO: move this to check the auto_tx class
    update_ptt();

    FSM_State next_state;
    switch ( cur_state ) {

        case FSM_IDLE:          next_state = fsm_idle(); break;
        case FSM_VOICE:         next_state = fsm_voice(); break;
        case FSM_PTT_KEY:       next_state = fsm_ptt_key(); break;
        case FSM_START_TX:      next_state = fsm_start_tx(); break;
        case FSM_TX:            next_state = fsm_tx(); break;
        default:                next_state = FSM_IDLE;

    }
    ESP_LOGI("fsm", "%d\n", next_state);
    cur_state = next_state;
    FSM_flag_reset_all();

}

// TODO: disable transmission for some time to allow for boot up
// TODO: move PTT active level somewhere so we can use
void fsm_init() {

    FSM_flag_reset_all();
    TX_enabled = 1;
    PTT_prev_level = 1;
    PTT_level = 1;
    cur_state = FSM_IDLE;
    
}

//=============================================== Static Functions ==========================================================

static FSM_State fsm_idle() {

    // if PTT pressed -> voice
    if ( was_ptt_pressed() ) { return FSM_VOICE; }
    // if auto transmission -> start transmission
    if( was_auto_tx_flagged() && is_tx_enabled() ) {

        key_ptt();
        return FSM_PTT_KEY;
    }
    // stay
    return FSM_IDLE;

}

// Just ensures there is enough time in between keying ptt and starting transmission
static FSM_State fsm_ptt_key() { return FSM_START_TX; }

static FSM_State fsm_voice() {

    // if PTT p
    if( was_ptt_released() ) { 

        if( is_tx_enabled() ) {

            key_ptt();
            reset_auto_tx_timer();
            return FSM_PTT_KEY; 
        }
        // released too soon
        return FSM_IDLE;

    }
    // ignore automatic tx in this case
    return FSM_VOICE;

}

static FSM_State fsm_start_tx() {

    // start transmitting and disable further transmits
    tx_gps();
    // TODO: add timer to tx or ptt and remove this; also would add an fsm end tx?
    //TX_enabled = 0;
    return FSM_TX;
}

static FSM_State fsm_tx() {

    if( was_ptt_disabled() ) { disable_tx(); return FSM_IDLE; }
    return FSM_TX;

}

// TODO: instead of creating timer to turn off ptt, do it for as long as you need then manually turn off once ptt done!!
// add an fsm_end_tx() state