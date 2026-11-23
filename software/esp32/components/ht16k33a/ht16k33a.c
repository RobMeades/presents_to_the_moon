/*
 * Copyright 2026 Rob Meades
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 * @brief Implementation of the HT16K33A 14-segment LED display driver API.
 * Written by DeepSeek based on the pattern of tmc5160.c
 */

#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sys/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "fgr_util.h"
#include "ht16k33a.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Logging prefix
#define TAG "ht16k33a"

// Default scroll period in milliseconds
#define SCROLL_DEFAULT_MS 250

// HT16K33A command bytes (single-byte commands)
#define CMD_DIMMING         0xE0
#define CMD_OSCILLATOR_ON   0x21
#define CMD_OSCILLATOR_OFF  0x20
#define CMD_DISPLAY_ON      0x81
#define CMD_DISPLAY_OFF     0x80
#define CMD_DISPLAY_RAM     0x00  // followed by 16 bytes of RAM

// Number of RAM bytes per unit (8 digits x 2 bytes; we use 4 digits)
#define RAM_BYTES_PER_UNIT  16

// HT16K33A 14-segment digit bit assignments, confirmed by hardware test.
// The 14 segments divide into:
//   bits 0..5   - the six outer ring segments (top, top-right,
//                 bottom-right, bottom, bottom-left, top-left)
//   bits 6..7   - the two inner horizontal segments, left then right
//   bits 8..10  - the three inner top strokes, left to right
//   bits 11..13 - the three inner bottom strokes, left to right
//   bits 14     - decimal point
#define SEG_OUTER_TOP          (1u << 0)
#define SEG_OUTER_TOP_RIGHT    (1u << 1)
#define SEG_OUTER_BOTTOM_RIGHT (1u << 2)
#define SEG_OUTER_BOTTOM       (1u << 3)
#define SEG_OUTER_BOTTOM_LEFT  (1u << 4)
#define SEG_OUTER_TOP_LEFT     (1u << 5)

#define SEG_INNER_MID_LEFT  (1u << 6)
#define SEG_INNER_MID_RIGHT (1u << 7)

#define SEG_INNER_TOP_LEFT  (1u << 8)   // top-left diagonal
#define SEG_INNER_TOP_MID   (1u << 9)   // top-middle vertical
#define SEG_INNER_TOP_RIGHT (1u << 10)  // top-right diagonal

#define SEG_INNER_BOTTOM_LEFT  (1u << 11)  // bottom-left diagonal
#define SEG_INNER_BOTTOM_MID   (1u << 12)  // bottom-middle vertical
#define SEG_INNER_BOTTOM_RIGHT (1u << 13)  // bottom-right diagonal

#define SEG_DP (1u << 14)

// The complete middle bar (two halves) is the natural "G" of a
// conventional 7-segment letter, but on this display the middle bar
// is drawn as the two inner segments, not as an outer-ring segment.
#define SEG_G (SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT)

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// One physical HT16K33A unit.
typedef struct {
    uint8_t  address;
    i2c_master_dev_handle_t dev_handle; // ESP-IDF v5 device handle
    uint16_t ram[HT16K33A_DIGITS_PER_UNIT];
    bool     present;
} unit_t;

// A logical HT16K33A display spanning one or more units.
struct ht16k33a_display {
    unit_t   units[HT16K33A_MAX_UNITS];
    size_t   num_units;
    size_t   num_digits;
    char     text[HT16K33A_MAX_TEXT_LENGTH];
    size_t   text_len;

    // Pre-rendered glyph per visible stream position, with the DP
    // already folded in.  r_len is text_len after DP folding.
    uint16_t r_text[HT16K33A_MAX_TEXT_LENGTH];
    size_t   r_len;

    size_t   scroll_offset;
    int8_t   scroll_direction;   // +1 or -1, for BOUNCE
    bool     bounced;            // true once BOUNCE has reversed

    ht16k33a_scroll_mode_t  scroll_mode;
    ht16k33a_scroll_end_t   scroll_end;
    ht16k33a_brightness_t   brightness;
    uint32_t                scroll_period_ms;
    bool                    paused;
    bool                    display_on;
    ht16k33a_scroll_done_t  done_callback;
    void                   *done_user_data;
    esp_timer_handle_t      scroll_timer;
    SLIST_ENTRY(ht16k33a_display) next;
};

SLIST_HEAD(ht16k33a_display_list_t, ht16k33a_display);

typedef struct {
    SemaphoreHandle_t lock;
    i2c_master_bus_handle_t bus_handle; // v5 bus handle
    bool bus_initialised;
    int32_t clock_hz;
    struct ht16k33a_display_list_t display_list;
} context_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// The context.
static context_t g_context = {0};

// 14-segment font, indexed by ASCII code minus 0x20.  Each entry is
// the bitmap of segments to light, built from the named macros above.
// Glyphs are chosen for legibility on a 14-segment display: where the
// inner strokes improve recognisability (K, M, N, T, V, W, X, Y, Z)
// they are used; where they add nothing (A, O, etc.) the outer ring
// alone is sufficient.
static const uint16_t g_font[] = {
    // 0x20 ' '
    [' '  - 0x20] = 0,
    // 0x21 '!'
    ['!'  - 0x20] = SEG_INNER_TOP_MID | SEG_INNER_BOTTOM_MID,
    // 0x22 '"'
    ['"'  - 0x20] = SEG_OUTER_TOP_LEFT | SEG_OUTER_TOP_RIGHT,
    // 0x23 '#'
    ['#'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT |
                    SEG_INNER_TOP_LEFT | SEG_INNER_TOP_MID |
                    SEG_INNER_TOP_RIGHT | SEG_INNER_BOTTOM_LEFT |
                    SEG_INNER_BOTTOM_MID | SEG_INNER_BOTTOM_RIGHT,
    // 0x24 '$'
    ['$'  - 0x20] = SEG_OUTER_TOP_LEFT | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT |
                    SEG_INNER_TOP_MID | SEG_INNER_BOTTOM_MID,
    // 0x25 '%'
    ['%'  - 0x20] = SEG_OUTER_TOP_LEFT | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM_LEFT,
    // 0x26 '&'
    ['&'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_LEFT |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_BOTTOM |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT |
                    SEG_INNER_BOTTOM_RIGHT,
    // 0x27 '\''
    ['\'' - 0x20] = SEG_OUTER_TOP_RIGHT,
    // 0x28 '('
    ['('  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x29 ')'
    [')'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_TOP_RIGHT,
    // 0x2A '*'
    ['*'  - 0x20] = SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT |
                    SEG_INNER_TOP_LEFT |
                    SEG_INNER_TOP_RIGHT | SEG_INNER_BOTTOM_LEFT |
                    SEG_INNER_BOTTOM_RIGHT,
    // 0x2B '+'
    ['+'  - 0x20] = SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT |
                    SEG_INNER_TOP_MID | SEG_INNER_BOTTOM_MID,
    // 0x2C ','
    [','  - 0x20] = SEG_OUTER_BOTTOM,
    // 0x2D '-'
    ['-'  - 0x20] = SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x2E '.'
    ['.'  - 0x20] = SEG_DP,
    // 0x2F '/'
    ['/'  - 0x20] = SEG_INNER_TOP_RIGHT | SEG_INNER_BOTTOM_LEFT,
    // 0x30 '0'
    ['0'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x31 '1'
    ['1'  - 0x20] = SEG_OUTER_TOP_RIGHT | SEG_OUTER_BOTTOM_RIGHT,
    // 0x32 '2'
    ['2'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM | SEG_OUTER_BOTTOM_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x33 '3'
    ['3'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x34 '4'
    ['4'  - 0x20] = SEG_OUTER_TOP_RIGHT | SEG_OUTER_BOTTOM_RIGHT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x35 '5'
    ['5'  - 0x20] = SEG_OUTER_TOP |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x36 '6'
    ['6'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM_RIGHT |
                    SEG_OUTER_BOTTOM | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x37 '7'
    ['7'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT,
    // 0x38 '8'
    ['8'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x39 '9'
    ['9'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x3A ':'
    [':'  - 0x20] = SEG_DP,
    // 0x3B ';' (same as ':')
    [';'  - 0x20] = SEG_DP,
    // 0x3C '<'
    ['<'  - 0x20] = SEG_INNER_TOP_RIGHT | SEG_INNER_BOTTOM_RIGHT,
    // 0x3D '='
    ['='  - 0x20] = SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT |
                    SEG_OUTER_BOTTOM,
    // 0x3E '>'
    ['>'  - 0x20] = SEG_INNER_TOP_LEFT | SEG_INNER_BOTTOM_LEFT,
    // 0x3F '?'
    ['?'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_INNER_MID_RIGHT | SEG_INNER_BOTTOM_MID,
    // 0x40 '@'
    ['@'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM_RIGHT |
                    SEG_OUTER_BOTTOM | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_RIGHT,
    // 0x41 'A'
    ['A'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x42 'B'
    ['B'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_RIGHT,
    // 0x43 'C'
    ['C'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x44 'D'
    ['D'  - 0x20] = SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_RIGHT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x45 'E'
    ['E'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x46 'F'
    ['F'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x47 'G'
    ['G'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM_RIGHT |
                    SEG_OUTER_BOTTOM | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_RIGHT,
    // 0x48 'H'
    ['H'  - 0x20] = SEG_OUTER_TOP_RIGHT | SEG_OUTER_BOTTOM_RIGHT |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x49 'I'
    ['I'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_INNER_TOP_MID | SEG_INNER_BOTTOM_MID,
    // 0x4A 'J'
    ['J'  - 0x20] = SEG_OUTER_TOP_LEFT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT,
    // 0x4B 'K'
    ['K'  - 0x20] = SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_TOP_RIGHT | SEG_INNER_BOTTOM_RIGHT,
    // 0x4C 'L'
    ['L'  - 0x20] = SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x4D 'M'
    ['M'  - 0x20] = SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_TOP_LEFT | SEG_INNER_TOP_RIGHT,
    // 0x4E 'N'
    ['N'  - 0x20] = SEG_OUTER_TOP_RIGHT | SEG_OUTER_BOTTOM_RIGHT |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_TOP_LEFT | SEG_INNER_BOTTOM_RIGHT,
    // 0x4F 'O'
    ['O'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x50 'P'
    ['P'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x51 'Q'
    ['Q'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_BOTTOM_RIGHT,
    // 0x52 'R'
    ['R'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT |
                    SEG_INNER_BOTTOM_RIGHT,
    // 0x53 'S'
    ['S'  - 0x20] = SEG_OUTER_TOP |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x54 'T'
    ['T'  - 0x20] = SEG_OUTER_TOP |
                    SEG_INNER_TOP_MID | SEG_INNER_BOTTOM_MID,
    // 0x55 'U'
    ['U'  - 0x20] = SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT | SEG_OUTER_TOP_RIGHT,
    // 0x56 'V'
    ['V'  - 0x20] = SEG_INNER_BOTTOM_LEFT | SEG_INNER_BOTTOM_RIGHT |
                    SEG_OUTER_TOP_LEFT | SEG_OUTER_TOP_RIGHT,
    // 0x57 'W'
    ['W'  - 0x20] = SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM_LEFT |
                    SEG_INNER_BOTTOM_LEFT | SEG_INNER_BOTTOM_RIGHT |
                    SEG_OUTER_TOP_LEFT | SEG_OUTER_TOP_RIGHT,
    // 0x58 'X'
    ['X'  - 0x20] = SEG_INNER_TOP_LEFT | SEG_INNER_TOP_RIGHT |
                    SEG_INNER_BOTTOM_LEFT | SEG_INNER_BOTTOM_RIGHT,
    // 0x59 'Y'
    ['Y'  - 0x20] = SEG_INNER_TOP_LEFT | SEG_INNER_TOP_RIGHT |
                    SEG_INNER_BOTTOM_MID,
    // 0x5A 'Z'
    ['Z'  - 0x20] = SEG_OUTER_TOP | SEG_INNER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM | SEG_INNER_BOTTOM_LEFT,
    // 0x5B '[' (same as '(')
    ['['  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x5C '\\'
    ['\\' - 0x20] = SEG_INNER_TOP_LEFT | SEG_INNER_BOTTOM_RIGHT,
    // 0x5D ']' (same as ')')
    [']'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_TOP_RIGHT,
    // 0x5E '^'
    ['^'  - 0x20] = SEG_INNER_BOTTOM_LEFT | SEG_INNER_BOTTOM_RIGHT,
    // 0x5F '_'
    ['_'  - 0x20] = SEG_OUTER_BOTTOM,
    // 0x60 '`'
    ['`'  - 0x20] = SEG_OUTER_TOP_LEFT,
    // 0x61 'a' (same as 'A')
    ['a'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x62 'b' (same as 'B')
    ['b'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_RIGHT,
    // 0x63 'c' (same as 'C')
    ['c'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x64 'd' (same as 'D')
    ['d'  - 0x20] = SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_RIGHT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x65 'e' (same as 'E')
    ['e'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x66 'f' (same as 'F')
    ['f'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x67 'g' (same as 'G')
    ['g'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM_RIGHT |
                    SEG_OUTER_BOTTOM | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_RIGHT,
    // 0x68 'h' (same as 'H')
    ['h'  - 0x20] = SEG_OUTER_TOP_RIGHT | SEG_OUTER_BOTTOM_RIGHT |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x69 'i' (same as 'I')
    ['i'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_INNER_TOP_MID | SEG_INNER_BOTTOM_MID,
    // 0x6A 'j' (same as 'J')
    ['j'  - 0x20] = SEG_OUTER_TOP_LEFT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT,
    // 0x6B 'k' (same as 'K')
    ['k'  - 0x20] = SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_TOP_RIGHT | SEG_INNER_BOTTOM_RIGHT,
    // 0x6C 'l' (same as 'L')
    ['l'  - 0x20] = SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x6D 'm' (same as 'M')
    ['m'  - 0x20] = SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_TOP_LEFT | SEG_INNER_TOP_RIGHT,
    // 0x6E 'n' (same as 'N')
    ['n'  - 0x20] = SEG_OUTER_TOP_RIGHT | SEG_OUTER_BOTTOM_RIGHT |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_TOP_LEFT | SEG_INNER_BOTTOM_RIGHT,
    // 0x6F 'o' (same as 'O')
    ['o'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x70 'p' (same as 'P')
    ['p'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x71 'q' (same as 'Q')
    ['q'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_BOTTOM_RIGHT,
    // 0x72 'r' (same as 'R')
    ['r'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT |
                    SEG_INNER_BOTTOM_RIGHT,
    // 0x73 's' (same as 'S')
    ['s'  - 0x20] = SEG_OUTER_TOP |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_TOP_LEFT |
                    SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT,
    // 0x74 't' (same as 'T')
    ['t'  - 0x20] = SEG_OUTER_TOP |
                    SEG_INNER_TOP_MID | SEG_INNER_BOTTOM_MID,
    // 0x75 'u' (same as 'U')
    ['u'  - 0x20] = SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT |
                    SEG_OUTER_TOP_LEFT | SEG_OUTER_TOP_RIGHT,
    // 0x76 'v' (same as 'V')
    ['v'  - 0x20] = SEG_INNER_BOTTOM_LEFT | SEG_INNER_BOTTOM_RIGHT |
                    SEG_OUTER_TOP_LEFT | SEG_OUTER_TOP_RIGHT,
    // 0x77 'w' (same as 'W')
    ['w'  - 0x20] = SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_BOTTOM_LEFT |
                    SEG_INNER_BOTTOM_LEFT | SEG_INNER_BOTTOM_RIGHT |
                    SEG_OUTER_TOP_LEFT | SEG_OUTER_TOP_RIGHT,
    // 0x78 'x' (same as 'X')
    ['x'  - 0x20] = SEG_INNER_TOP_LEFT | SEG_INNER_TOP_RIGHT |
                    SEG_INNER_BOTTOM_LEFT | SEG_INNER_BOTTOM_RIGHT,
    // 0x79 'y' (same as 'Y')
    ['y'  - 0x20] = SEG_INNER_TOP_LEFT | SEG_INNER_TOP_RIGHT |
                    SEG_INNER_BOTTOM_MID,
    // 0x7A 'z' (same as 'Z')
    ['z'  - 0x20] = SEG_OUTER_TOP | SEG_INNER_TOP_RIGHT |
                    SEG_OUTER_BOTTOM | SEG_INNER_BOTTOM_LEFT,
    // 0x7B '{' (same as '(')
    ['{'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_LEFT | SEG_OUTER_TOP_LEFT,
    // 0x7C '|'
    ['|'  - 0x20] = SEG_INNER_TOP_MID | SEG_INNER_BOTTOM_MID,
    // 0x7D '}' (same as '(')
    ['}'  - 0x20] = SEG_OUTER_TOP | SEG_OUTER_BOTTOM |
                    SEG_OUTER_BOTTOM_RIGHT | SEG_OUTER_TOP_RIGHT,
    // 0x7E '~' (same as '=')
    ['~'  - 0x20] = SEG_INNER_MID_LEFT | SEG_INNER_MID_RIGHT |
                    SEG_OUTER_BOTTOM,
};

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

static uint16_t char_to_segments(char c, bool dp)
{
    uint16_t seg = 0;
    if ((uint8_t) c >= 0x20 && (uint8_t) c <= 0x7F) {
        seg = g_font[(uint8_t) c - 0x20];
    }

    // Show unknown non-whitespace characters as an underscore
    if ((seg == 0) && (c != ' ') && (c != '\t')) {
        seg = SEG_OUTER_BOTTOM;
    }

    if (dp) {
        seg |= SEG_DP;
    }
    return seg;
}

// Pre-render the text into r_text[] with any trailing '.', ':', '/'
// or '`' folded into the preceding character as DP.  Returns the
// number of visible stream positions (r_len).
// IMPORTANT: the context must be locked before this is called.
static size_t render_text(ht16k33a_display_t *display)
{
    size_t out = 0;

    for (size_t x = 0; x < display->text_len; x++) {
        bool dp = false;
        char n = display->text[x];
        while ((x + 1 < display->text_len) &&
               ((display->text[x + 1] == '.') ||
                (display->text[x + 1] == ':') ||
                (display->text[x + 1] == '/') ||
                (display->text[x + 1] == '`'))) {
            dp = true;
            x++;
        }
        display->r_text[out++] = char_to_segments(n, dp);
    }

    return out;
}

// Write a single command byte to a unit, v5 API.
// IMPORTANT: the context must be locked before this is called.
static int32_t unit_command(unit_t *unit, uint8_t command)
{
    if (!unit->dev_handle) {
        return -ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_master_transmit(unit->dev_handle,
                                        &command, 1,
                                        pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C command 0x%02x to 0x%02x failed (%s).",
                 command, unit->address, esp_err_to_name(err));
        unit->present = false;
    }
    return -err;
}

// Push the current RAM of all units over I2C, v5 API.
// IMPORTANT: the context must be locked before this is called.
static int32_t ram_flush(ht16k33a_display_t *display)
{
    int32_t err = ESP_OK;

    for (size_t x = 0; (x < display->num_units) && (err == ESP_OK); x++) {
        unit_t *unit = &display->units[x];

        if (unit->present && unit->dev_handle) {
            uint8_t buffer[1 + RAM_BYTES_PER_UNIT];
            buffer[0] = CMD_DISPLAY_RAM;
            for (size_t d = 0; d < HT16K33A_DIGITS_PER_UNIT; d++) {
                buffer[1 + d * 2] = (uint8_t) (unit->ram[d] & 0xff);
                buffer[2 + d * 2] = (uint8_t) ((unit->ram[d] >> 8) & 0xff);
            }
            esp_err_t i2c_err = i2c_master_transmit(unit->dev_handle,
                                                    buffer, sizeof(buffer),
                                                    1000);
            if (i2c_err != ESP_OK) {
                ESP_LOGE(TAG, "I2C RAM flush to 0x%02x failed (%s).",
                         unit->address, esp_err_to_name(i2c_err));
                unit->present = false;
                err = -i2c_err;
            }
        }
    }

    return err;
}

// Blank the RAM of a display.
// IMPORTANT: the context must be locked before this is called.
static void ram_blank(ht16k33a_display_t *display)
{
    for (size_t x = 0; x < display->num_units; x++) {
        memset(display->units[x].ram, 0, sizeof(display->units[x].ram));
    }
}

// Render the ticker window at stream position `position`.  The
// stream is r_text[] followed by num_digits blank positions, so
// that a gap of one display-width sits between successive copies
// of the text.  Digit k of the display shows
//   r_text[(position + k) % L]  if that index < r_len, else blank.
// IMPORTANT: the context must be locked before this is called.
static void ram_render(ht16k33a_display_t *display, size_t position)
{
    size_t length = display->r_len + display->num_digits;

    ram_blank(display);

    if (length > 0) {
        for (size_t x = 0; x < display->num_digits; x++) {
            size_t index = (position + x) % length;
            if (index < display->r_len) {
                size_t unit_index = x / HT16K33A_DIGITS_PER_UNIT;
                size_t unit_digit = x % HT16K33A_DIGITS_PER_UNIT;
                display->units[unit_index].ram[unit_digit] = display->r_text[index];
            }
        }
    }
}

// Stop the scroll timer.
// IMPORTANT: the context must be locked before this is called.
static void scroll_stop(ht16k33a_display_t *display)
{
    if (display->scroll_timer) {
        esp_timer_stop(display->scroll_timer);
        esp_timer_delete(display->scroll_timer);
        display->scroll_timer = NULL;
    }
    display->paused = false;
}

// Advance the scroll one step and re-render.  Called from the timer.
// IMPORTANT: the context must be locked before this is called.
static void scroll_tick(ht16k33a_display_t *display)
{
    size_t length = display->r_len + display->num_digits;

    if (display->paused || (length == 0) ||
        (display->r_len <= display->num_digits)) {
        // Nothing to do
    } else {
        // The "fully visible" resting point, shared by STOP_AT_END
        // and the return leg of BOUNCE.
        size_t rest_end = display->r_len - display->num_digits;
        // The BOUNCE jog point: one position past rest_end.
        size_t jog_end = rest_end + 1;

        if (display->scroll_mode == HT16K33A_SCROLL_LEFT) {

            if (display->scroll_direction > 0) {
                // Outbound leg.
                if (display->scroll_offset + 1 >= length) {
                    // Reached the far end of the ticker stream.
                    if (display->scroll_end == HT16K33A_SCROLL_WRAP) {
                        display->scroll_offset = 0;
                    } else if (display->scroll_end == HT16K33A_SCROLL_BOUNCE) {
                        // Defensive: should not happen because the
                        // jog point is checked below before we get
                        // here.
                        display->scroll_direction = -1;
                        display->bounced = true;
                    } else {
                        // STOP_AT_END
                        if (display->done_callback) {
                            display->done_callback(display,
                                                   display->done_user_data);
                        }
                        scroll_stop(display);
                    }
                } else if ((display->scroll_end == HT16K33A_SCROLL_BOUNCE) &&
                           (display->scroll_offset >= jog_end)) {
                    // We are at the jog point: reverse direction and
                    // step back one.  The next tick sees direction -1
                    // and steps back to rest_end, then stops.
                    display->scroll_direction = -1;
                    display->bounced = true;
                    display->scroll_offset--;
                } else {
                    display->scroll_offset++;
                }
            } else if (display->bounced) {
                // Return leg of a bounce: step back one, then stop.
                if (display->scroll_offset > rest_end) {
                    display->scroll_offset--;
                } else {
                    if (display->done_callback) {
                        display->done_callback(display,
                                               display->done_user_data);
                    }
                    scroll_stop(display);
                }
            } else {
                // scroll_direction < 0 but no bounce has occurred;
                // treat as idle.
            }

        } else {  // HT16K33A_SCROLL_RIGHT

            if (display->scroll_direction < 0) {
                // Outbound leg.
                if (display->scroll_offset + 1 >= length) {
                    if (display->scroll_end == HT16K33A_SCROLL_WRAP) {
                        display->scroll_offset = 0;
                    } else if (display->scroll_end == HT16K33A_SCROLL_BOUNCE) {
                        display->scroll_direction = +1;
                        display->bounced = true;
                    } else {
                        if (display->done_callback) {
                            display->done_callback(display,
                                                   display->done_user_data);
                        }
                        scroll_stop(display);
                    }
                } else if ((display->scroll_end == HT16K33A_SCROLL_BOUNCE) &&
                           (display->scroll_offset >= jog_end)) {
                    display->scroll_direction = +1;
                    display->bounced = true;
                    display->scroll_offset--;
                } else {
                    display->scroll_offset++;
                }
            } else if (display->bounced) {
                // Return leg of a bounce: step back one, then stop.
                if (display->scroll_offset > rest_end) {
                    display->scroll_offset--;
                } else {
                    if (display->done_callback) {
                        display->done_callback(display,
                                               display->done_user_data);
                    }
                    scroll_stop(display);
                }
            } else {
                // Idle.
            }
        }

        ram_render(display, display->scroll_offset);
        ram_flush(display);
    }
}

// Timer callback: takes the lock and calls scroll_tick().
static void scroll_timer_callback(void *arg)
{
    ht16k33a_display_t *display = (ht16k33a_display_t *) arg;

    if (g_context.lock) {
        CONTEXT_LOCK(g_context.lock, "scroll_timer_callback()");
        scroll_tick(display);
        CONTEXT_UNLOCK(g_context.lock, "scroll_timer_callback()");
    }
}

// Start (or restart) the scroll timer.
// IMPORTANT: the context must be locked before this is called.
static int32_t scroll_start(ht16k33a_display_t *display)
{
    int32_t err = ESP_OK;

    scroll_stop(display);

    if ((display->scroll_mode != HT16K33A_SCROLL_NONE) &&
        (display->r_len > display->num_digits)) {
        esp_timer_create_args_t args = {
            .callback = scroll_timer_callback,
            .arg = display,
            .name = "ht16k33a_scroll"
        };
        esp_err_t timer_err = esp_timer_create(&args, &display->scroll_timer);
        if (timer_err == ESP_OK) {
            uint32_t period_us = display->scroll_period_ms * 1000;
            if (period_us == 0) {
                period_us = SCROLL_DEFAULT_MS * 1000;
            }
            timer_err = esp_timer_start_periodic(display->scroll_timer,
                                                 period_us);
            if (timer_err != ESP_OK) {
                esp_timer_delete(display->scroll_timer);
                display->scroll_timer = NULL;
            }
        }
        if (timer_err != ESP_OK) {
            ESP_LOGE(TAG, "failed to start scroll timer (%s).",
                     esp_err_to_name(timer_err));
            err = -timer_err;
        }
    }

    return err;
}

// Turn display on or off.
// IMPORTANT the context should be locked before this is called.
static int32_t display_on(ht16k33a_display_t *display, bool on_not_off)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (display) {
        err = ESP_OK;
        for (size_t x = 0; (x < display->num_units) && (err == ESP_OK); x++) {
            unit_t *unit = &display->units[x];
            if (unit->present) {
                if (on_not_off) {
                    err = unit_command(unit, CMD_DISPLAY_ON);
                    if (err == ESP_OK) {
                        err = unit_command(unit, CMD_DIMMING | (display->brightness & 0x0f));
                    }
                } else {
                    err = unit_command(unit, CMD_DISPLAY_OFF);
                }
            }
        }
        if (err == ESP_OK) {
            display->display_on = on_not_off;
        }
    }

    return err;
}

// Clean up a display.
// IMPORTANT the context should be locked before this is called.
static void clean_up(ht16k33a_display_t *display)
{
    if (display) {
        scroll_stop(display);

        for (size_t x = 0; x < display->num_units; x++) {
            if (display->units[x].dev_handle) {
                i2c_master_bus_rm_device(display->units[x].dev_handle);
                display->units[x].dev_handle = NULL;
            }
        }

        ht16k33a_display_t *iter;
        ht16k33a_display_t *prev = NULL;
        SLIST_FOREACH(iter, &g_context.display_list, next) {
            if (iter == display) {
                if (prev == NULL) {
                    SLIST_REMOVE_HEAD(&g_context.display_list, next);
                } else {
                    SLIST_REMOVE_AFTER(prev, next);
                }
                break;
            }
            prev = iter;
        }
        free(display);
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Initialise the I2C master bus.
int32_t ht16k33a_init(int32_t i2c_port, int32_t sda_pin,
                      int32_t scl_pin, int32_t clock_hz)
{
    int32_t err = ESP_OK;

    if (!g_context.lock) {
        g_context.lock = xSemaphoreCreateMutex();
        if (g_context.lock) {
            SLIST_INIT(&g_context.display_list);
        } else {
            err = -ESP_ERR_NO_MEM;
        }
    }

    if (err == ESP_OK) {
        CONTEXT_LOCK(g_context.lock, "ht16k33a_init()");

        if (!g_context.bus_initialised) {
            ESP_LOGI(TAG, "starting I2C %d at %d Hz with SDA pin %d,"
                     " SCL pin %d.", i2c_port, clock_hz, sda_pin, scl_pin);

            i2c_master_bus_config_t bus_config = {
                .i2c_port = i2c_port,
                .sda_io_num = sda_pin,
                .scl_io_num = scl_pin,
                .clk_source = I2C_CLK_SRC_DEFAULT,
                .glitch_ignore_cnt = 7,
                .flags.enable_internal_pullup = true,
            };

            esp_err_t i2c_err = i2c_new_master_bus(&bus_config,
                                                   &g_context.bus_handle);
            if (i2c_err != ESP_OK) {
                ESP_LOGE(TAG, "I2C bus initialisation failed (%s).",
                         esp_err_to_name(i2c_err));
                err = -i2c_err;
            } else {
                g_context.bus_initialised = true;
                g_context.clock_hz = clock_hz;
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_init()");
    }

    return err;
}

// Create a display instance, adding one device handle per unit.
ht16k33a_display_t *ht16k33a_create(const uint8_t *address_list, size_t num_units)
{
    ht16k33a_display_t *display = NULL;

    if (g_context.bus_initialised && address_list && (num_units > 0) && (num_units <= HT16K33A_MAX_UNITS)) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_create()");

        int32_t err = ESP_ERR_NO_MEM;
        display = malloc(sizeof(*display));
        if (display) {
            memset(display, 0, sizeof(*display));
            err = ESP_OK;

            display->num_units = num_units;
            display->num_digits = num_units * HT16K33A_DIGITS_PER_UNIT;
            display->scroll_mode = HT16K33A_SCROLL_NONE;
            display->scroll_end = HT16K33A_SCROLL_WRAP;
            display->brightness = HT16K33A_BRIGHTNESS_15;
            display->scroll_period_ms = SCROLL_DEFAULT_MS;
            display->display_on = true;

            for (size_t x = 0; (x < display->num_units) && (err == ESP_OK); x++) {
                display->units[x].address = address_list[x];

                i2c_device_config_t dev_config = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = address_list[x],
                    .scl_speed_hz = g_context.clock_hz,
                };

                esp_err_t i2c_err = i2c_master_bus_add_device(g_context.bus_handle,
                                                              &dev_config,
                                                              &display->units[x].dev_handle);
                if (i2c_err == ESP_OK) {
                    err = i2c_master_probe(g_context.bus_handle, address_list[x], 1000);
                    ESP_LOGI(TAG, "Probe 0x%02x: %s", address_list[x], esp_err_to_name(err));

                    display->units[x].present = true;
                    err = unit_command(&display->units[x], CMD_OSCILLATOR_ON);
                    if (err == ESP_OK) {
                        err = unit_command(&display->units[x], CMD_DISPLAY_ON);
                        if (err == ESP_OK) {
                            err = unit_command(&display->units[x], CMD_DIMMING | (display->brightness & 0x0f));
                        }
                    }
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "unit 0x%02x did not respond.",
                                address_list[x]);
                    }
                } else {
                    ESP_LOGE(TAG, "failed to add device 0x%02x (%s).",
                             address_list[x], esp_err_to_name(i2c_err));
                    err = -i2c_err;
                }
            }

            if (err == ESP_OK) {
                err = ram_flush(display);
            }
        }

        if (err == ESP_OK) {
            SLIST_INSERT_HEAD(&g_context.display_list, display, next);
            ESP_LOGI(TAG, "display created with %d unit(s).", num_units);
        } else {
            clean_up(display);
            display = NULL;
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_create()");
    }

    return display;
}

// Destroy a display instance.
void ht16k33a_destroy(ht16k33a_display_t *display)
{
    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_destroy()");
        clean_up(display);
        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_destroy()");
    }
}

// Deinitialise the driver.
void ht16k33a_deinit()
{
    if (g_context.bus_initialised) {
        CONTEXT_LOCK(g_context.lock, "ht16k33a_deinit()");

        while (!SLIST_EMPTY(&g_context.display_list)) {
            ht16k33a_display_t *p = SLIST_FIRST(&g_context.display_list);
            clean_up(p);
        }

        if (g_context.bus_handle) {
            i2c_del_master_bus(g_context.bus_handle);
            g_context.bus_handle = NULL;
        }
        g_context.bus_initialised = false;

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_deinit()");
    }
    // The semaphore will be re-used
}

// Set brightness.
int32_t ht16k33a_set_brightness(ht16k33a_display_t *display,
                                ht16k33a_brightness_t brightness)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_set_brightness()");

        err = -ESP_ERR_INVALID_ARG;
        if (display && (brightness >= HT16K33A_BRIGHTNESS_0) && (brightness <= HT16K33A_BRIGHTNESS_15)) {
            err = ESP_OK;
            for (size_t x = 0; (x < display->num_units) && (err == ESP_OK); x++) {
                unit_t *unit = &display->units[x];
                if (unit->present) {
                    err = unit_command(&display->units[x], CMD_DIMMING | (display->brightness & 0x0f));
                }
            }
            if (err == ESP_OK) {
                display->brightness = brightness;
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_set_brightness()");
    }

    return err;
}

// Get brightness.
int32_t ht16k33a_get_brightness(ht16k33a_display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_get_brightness()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            err = display->brightness;
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_get_brightness()");
    }

    return err;
}

// Turn the display on.
int32_t ht16k33a_set_display_on(ht16k33a_display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_set_display_on()");
        err = display_on(display, true);
        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_set_display_on()");
    }

    return err;
}

// Turn the display off.
int32_t ht16k33a_set_display_off(ht16k33a_display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_set_display_off()");
        err = display_on(display, false);
        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_set_display_off()");
    }

    return err;
}

// Set the text.
int32_t ht16k33a_set_text(ht16k33a_display_t *display, const char *text)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_set_text()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            // Copy the text, truncating at the buffer limit
            display->text_len = 0;
            if (text) {
                size_t len = strlen(text);
                if (len >= HT16K33A_MAX_TEXT_LENGTH) {
                    len = HT16K33A_MAX_TEXT_LENGTH - 1;
                    ESP_LOGW(TAG, "text truncated to %d characters.",
                             HT16K33A_MAX_TEXT_LENGTH - 1);
                }
                memcpy(display->text, text, len);
                display->text[len] = '\0';
                display->text_len = len;
            }

            // Pre-render the text so the scroll engine only has to
            // walk r_text[], and reset the scroll state.
            display->r_len = render_text(display);
            display->scroll_offset = 0;
            display->scroll_direction = +1;
            display->bounced = false;

            // Render the initial window
            err = ram_flush(display);  // ensure clean state
            ram_render(display, display->scroll_offset);
            err = ram_flush(display);

            if (err == ESP_OK) {
                if (display->scroll_mode != HT16K33A_SCROLL_NONE) {
                    err = scroll_start(display);
                }
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_set_text()");
    }

    return err;
}

// Set text fixed (no scrolling).
int32_t ht16k33a_set_text_fixed(ht16k33a_display_t *display, const char *text)
{
    int32_t err = ht16k33a_set_scroll_mode(display, HT16K33A_SCROLL_NONE);
    if (err == ESP_OK) {
        err = ht16k33a_set_text(display, text);
    }

    return err;
}

// Set scroll mode.
int32_t ht16k33a_set_scroll_mode(ht16k33a_display_t *display,
                                 ht16k33a_scroll_mode_t mode)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_set_scroll_mode()");

        err = -ESP_ERR_INVALID_ARG;
        if (display && (mode <= HT16K33A_SCROLL_RIGHT)) {
            err = ESP_OK;
            display->scroll_mode = mode;
            if (mode == HT16K33A_SCROLL_NONE) {
                // Stop scrolling and re-render at the start
                scroll_stop(display);
                ram_render(display, 0);
                err = ram_flush(display);
            } else if (display->r_len > 0) {
                // (Re)start scrolling if we have text
                display->scroll_offset = 0;
                display->scroll_direction = +1;
                display->bounced = false;
                err = scroll_start(display);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_set_scroll_mode()");
    }

    return err;
}

// Get scroll mode.
int32_t ht16k33a_get_scroll_mode(ht16k33a_display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_get_scroll_mode()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            err = display->scroll_mode;
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_get_scroll_mode()");
    }

    return err;
}

// Set end-of-scroll behaviour.
int32_t ht16k33a_set_scroll_end(ht16k33a_display_t *display,
                                ht16k33a_scroll_end_t mode)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_set_scroll_end()");

        err = -ESP_ERR_INVALID_ARG;
        if (display && (mode <= HT16K33A_SCROLL_STOP_AT_END)) {
            display->scroll_end = mode;
            err = ESP_OK;
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_set_scroll_end()");
    }

    return err;
}

// Get end-of-scroll behaviour.
int32_t ht16k33a_get_scroll_end(ht16k33a_display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_get_scroll_end()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            err = display->scroll_end;
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_get_scroll_end()");
    }

    return err;
}

// Set scroll rate.
int32_t ht16k33a_set_scroll_rate(ht16k33a_display_t *display,
                                 uint32_t milliseconds)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_set_scroll_rate()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            display->scroll_period_ms = milliseconds > 0 ?
                                        milliseconds : SCROLL_DEFAULT_MS;
            err = ESP_OK;
            // Restart if scrolling
            if (display->scroll_timer) {
                err = scroll_start(display);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_set_scroll_rate()");
    }

    return err;
}

// Pause scrolling.
int32_t ht16k33a_pause_scroll(ht16k33a_display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_pause_scroll()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            display->paused = true;
            err = ESP_OK;
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_pause_scroll()");
    }

    return err;
}

// Resume scrolling.
int32_t ht16k33a_resume_scroll(ht16k33a_display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_resume_scroll()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            display->paused = false;
            err = ESP_OK;
            // If a STOP_AT_END (or completed BOUNCE) stopped the timer,
            // restart it.
            if ((display->scroll_timer == NULL) &&
                (display->scroll_mode != HT16K33A_SCROLL_NONE) &&
                (display->r_len > display->num_digits)) {
                err = scroll_start(display);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_resume_scroll()");
    }

    return err;
}

// Set the scroll-done callback.
int32_t ht16k33a_set_scroll_done_callback(ht16k33a_display_t *display,
                                          ht16k33a_scroll_done_t callback,
                                          void *user_data)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_set_scroll_done_callback()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            display->done_callback = callback;
            display->done_user_data = user_data;
            err = ESP_OK;
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_set_scroll_done_callback()");
    }

    return err;
}

// Write immediate text (no scroll).
int32_t ht16k33a_write_immediate(ht16k33a_display_t *display, const char *text)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_write_immediate()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            scroll_stop(display);

            if (text) {
                // Render directly without touching display->text
                // (so scroll state is preserved for later)
                ram_blank(display);
                size_t digit = 0;
                for (size_t c = 0;
                     (text[c] != '\0') && (digit < display->num_digits);
                     c++, digit++) {
                    bool dp = false;
                    if ((text[c + 1] == '.') || (text[c + 1] == ':') ||
                        (text[c + 1] == '/') || (text[c + 1] == '`')) {
                        dp = true;
                    }
                    size_t unit_index = digit / HT16K33A_DIGITS_PER_UNIT;
                    size_t unit_digit = digit % HT16K33A_DIGITS_PER_UNIT;
                    display->units[unit_index].ram[unit_digit] = char_to_segments(text[c], dp);
                    if (dp) {
                        c++;
                    }
                }
            } else {
                ram_blank(display);
            }

            err = ram_flush(display);
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_write_immediate()");
    }

    return err;
}

// Clear the display.
int32_t ht16k33a_clear(ht16k33a_display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_clear()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            scroll_stop(display);
            display->text_len = 0;
            display->text[0] = '\0';
            display->r_len = 0;
            display->scroll_offset = 0;
            display->scroll_direction = +1;
            display->bounced = false;
            ram_blank(display);
            err = ram_flush(display);
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_clear()");
    }

    return err;
}

// Get the number of digits.
int32_t ht16k33a_get_num_digits(ht16k33a_display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.bus_initialised) {

        CONTEXT_LOCK(g_context.lock, "ht16k33a_get_num_digits()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            err = display->num_digits;
        }

        CONTEXT_UNLOCK(g_context.lock, "ht16k33a_get_num_digits()");
    }

    return err;
}

// End of file