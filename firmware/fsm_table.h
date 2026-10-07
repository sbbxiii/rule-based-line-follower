/**
 * @file fsm_table.h
 * @brief Finite State Machine (FSM) action definitions and O(1) state lookup table.
 * 
 * Mathematical Formulation:
 * State vector: S = [s1, s2, s3, s4, s5] in {0, 1}^5 (32 states)
 * Where:
 *   si = 0 denotes black line detected (low reflectance)
 *   si = 1 denotes white background detected (high reflectance)
 * 
 * Action space: A = { FORWARD, SLIGHT_LEFT, SLIGHT_RIGHT, SHARP_LEFT, SHARP_RIGHT, HALT, RECOVER }
 * 
 * Control law: Gamma(S) = g(f(S)), evaluated in O(1) time via precomputed lookup table.
 */

#ifndef FSM_TABLE_H
#define FSM_TABLE_H

#include <stdint.h>

// Action Set A
typedef enum {
    ACTION_FORWARD = 0,
    ACTION_SLIGHT_LEFT,
    ACTION_SLIGHT_RIGHT,
    ACTION_SHARP_LEFT,
    ACTION_SHARP_RIGHT,
    ACTION_HALT,
    ACTION_USE_LAST_STATE // Ambient/ambiguous fallback: Gamma(S_amb) = Gamma(S_t-1)
} RobotAction;

/**
 * 32-State Lookup Table f: S -> A
 * Index is calculated as:
 *   index = (s0 << 4) | (s1 << 3) | (s2 << 2) | (s3 << 1) | (s4 << 0)
 * where s0 is Far Left and s4 is Far Right.
 */
static const RobotAction STATE_LOOKUP_TABLE[32] = {
    /* 00000 ( 0) */ ACTION_USE_LAST_STATE, // All black (Crossroad/T-junction intersection: coast on momentum)
    /* 00001 ( 1) */ ACTION_SHARP_LEFT,     // Heavy left deviation
    /* 00010 ( 2) */ ACTION_SHARP_LEFT,
    /* 00011 ( 3) */ ACTION_SHARP_LEFT,
    /* 00100 ( 4) */ ACTION_FORWARD,
    /* 00101 ( 5) */ ACTION_FORWARD,
    /* 00110 ( 6) */ ACTION_SLIGHT_LEFT,
    /* 00111 ( 7) */ ACTION_SHARP_RIGHT,    // Line strongly on right (sensors s0,s1 dark)
    /* 01000 ( 8) */ ACTION_SHARP_LEFT,
    /* 01001 ( 9) */ ACTION_FORWARD,
    /* 01010 (10) */ ACTION_FORWARD,
    /* 01011 (11) */ ACTION_SLIGHT_RIGHT,
    /* 01100 (12) */ ACTION_SHARP_LEFT,
    /* 01101 (13) */ ACTION_SLIGHT_LEFT,
    /* 01110 (14) */ ACTION_FORWARD,
    /* 01111 (15) */ ACTION_SHARP_RIGHT,    // Far-right sharp curve
    /* 10000 (16) */ ACTION_SHARP_RIGHT,
    /* 10001 (17) */ ACTION_FORWARD,
    /* 10010 (18) */ ACTION_SLIGHT_RIGHT,
    /* 10011 (19) */ ACTION_SLIGHT_RIGHT,   // Line right-centered
    /* 10100 (20) */ ACTION_SLIGHT_LEFT,
    /* 10101 (21) */ ACTION_FORWARD,
    /* 10110 (22) */ ACTION_SLIGHT_LEFT,
    /* 10111 (23) */ ACTION_SLIGHT_RIGHT,   // Gentle veer right
    /* 11000 (24) */ ACTION_SHARP_LEFT,
    /* 11001 (25) */ ACTION_SLIGHT_LEFT,    // Line left-centered
    /* 11010 (26) */ ACTION_SLIGHT_LEFT,
    /* 11011 (27) */ ACTION_FORWARD,        // Nominal centered: [1, 1, 0, 1, 1]
    /* 11100 (28) */ ACTION_SHARP_LEFT,     // Sharp left corner
    /* 11101 (29) */ ACTION_SLIGHT_LEFT,    // Gentle veer left
    /* 11110 (30) */ ACTION_SHARP_LEFT,     // Far-left sharp curve
    /* 11111 (31) */ ACTION_USE_LAST_STATE  // All white (line lost): invoke direction memory
};

/**
 * Returns human-readable label for debugging serial stream.
 */
static inline const char* getActionName(RobotAction action) {
    switch (action) {
        case ACTION_FORWARD:      return "FORWARD";
        case ACTION_SLIGHT_LEFT:  return "SLIGHT_LEFT";
        case ACTION_SLIGHT_RIGHT: return "SLIGHT_RIGHT";
        case ACTION_SHARP_LEFT:   return "SHARP_LEFT";
        case ACTION_SHARP_RIGHT:  return "SHARP_RIGHT";
        case ACTION_HALT:         return "HALT";
        case ACTION_USE_LAST_STATE: return "MEMORY_FALLBACK";
        default:                  return "UNKNOWN";
    }
}

#endif // FSM_TABLE_H
