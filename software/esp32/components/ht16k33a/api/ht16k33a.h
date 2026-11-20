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

#ifndef _HT16K33A_H_
#define _HT16K33A_H_

/** @file
 * @brief The HT16K33A 14-segment LED display driver API.
 * Written by DeepSeek based on the pattern of tmc5160.h
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/** The maximum number of 4-digit units that can be chained. */
#define HT16K33A_MAX_UNITS 8

/** The number of 14-segment digits per unit. */
#define HT16K33A_DIGITS_PER_UNIT 4

/** The maximum number of characters in a text buffer, including
 * the null terminator.  This is not limited to the number of
 * visible digits: longer strings will scroll.
 */
#define HT16K33A_MAX_TEXT_LENGTH 256

/** I2C addresses of the HT16K33A: the base address is 0x70 and
 * the three address pins give 0x70 to 0x77.
 */
#define HT16K33A_I2C_ADDRESS_BASE 0x70

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* Display handle. */
typedef struct display display_t;

/** Display brightness, 0 (off) to 15 (maximum). */
typedef enum {
    HT16K33A_BRIGHTNESS_0 = 0,
    HT16K33A_BRIGHTNESS_1,
    HT16K33A_BRIGHTNESS_2,
    HT16K33A_BRIGHTNESS_3,
    HT16K33A_BRIGHTNESS_4,
    HT16K33A_BRIGHTNESS_5,
    HT16K33A_BRIGHTNESS_6,
    HT16K33A_BRIGHTNESS_7,
    HT16K33A_BRIGHTNESS_8,
    HT16K33A_BRIGHTNESS_9,
    HT16K33A_BRIGHTNESS_10,
    HT16K33A_BRIGHTNESS_11,
    HT16K33A_BRIGHTNESS_12,
    HT16K33A_BRIGHTNESS_13,
    HT16K33A_BRIGHTNESS_14,
    HT16K33A_BRIGHTNESS_15
} ht16k33a_brightness_t;

/** How the text should be presented when it is longer than the
 * number of available digits.
 */
typedef enum {
    HT16K33A_SCROLL_NONE,     /**< truncate at the display width */
    HT16K33A_SCROLL_LEFT,     /**< scroll right-to-left (text moves left) */
    HT16K33A_SCROLL_RIGHT     /**< scroll left-to-right (text moves right) */
} ht16k33a_scroll_mode_t;

/** What the scroll engine does when it reaches the end of the text.
 */
typedef enum {
    HT16K33A_SCROLL_WRAP,       /**< wrap round to the start and keep going */
    HT16K33A_SCROLL_BOUNCE,     /**< reverse direction at each end */
    HT16K33A_SCROLL_STOP_AT_END /**< stop with the last N digits visible */
} ht16k33a_scroll_end_t;

/** Called when the scroll engine reaches its end point (only
 * meaningful with HT16K33A_SCROLL_STOP_AT_END).  Runs in the
 * scroll timer/task context, so keep it short: set an event, notify
 * a task, don't block.  The display is NOT automatically blanked.
 */
typedef void (*ht16k33a_scroll_done_t) (display_t *display, void *user_data);

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

/** Initialise the interface to the HT16K33A.  Note that
 * ht16k33a_create() still needs to be called before a display
 * will respond.
 *
 * @param i2c_port   I2C port, e.g. I2C_NUM_0.
 * @param sda_pin    GPIO for I2C SDA.
 * @param scl_pin    GPIO for I2C SCL.
 * @param clock_hz   I2C clock frequency in Hz (400000 is typical).
 * @return           ESP_OK on success, else a negative value
 *                   from esp_err_t.
 */
int32_t ht16k33a_init(int32_t i2c_port, int32_t sda_pin,
                      int32_t scl_pin, int32_t clock_hz);

/** Create a display instance covering one or more HT16K33A units.
 *
 * The units are assumed to be daisy-chained in I2C address order:
 * the unit at index 0 is the leftmost four digits, index 1 the
 * next four to the right, and so on.
 *
 * @param address_list array of I2C addresses, one per unit, in
 *                     left-to-right display order; must contain
 *                     at least one entry.
 * @param num_units    number of units, 1 to HT16K33A_MAX_UNITS.
 * @return             the display instance, NULL on error.
 */
display_t *ht16k33a_create(const uint8_t *address_list, size_t num_units);

/** Destroy a display instance.
 *
 * @param display the display instance that was returned by
 *                ht16k33a_create().
 */
void ht16k33a_destroy(display_t *display);

/** Deinitialise the interface to the HT16K33A.  Any display
 * instances will be removed as if ht16k33a_destroy() had been
 * called on them.
 */
void ht16k33a_deinit();

/** Set the brightness of all units of a display.
 *
 * @param display    the display instance.
 * @param brightness the desired brightness, 0 to 15.
 * @return           zero on success else negative error code
 *                   from esp_err_t.
 */
int32_t ht16k33a_set_brightness(display_t *display,
                                ht16k33a_brightness_t brightness);

/** Get the brightness of a display.
 *
 * @param display the display instance.
 * @return        the brightness, else negative error code
 *                from esp_err_t.
 */
int32_t ht16k33a_get_brightness(display_t *display);

/** Turn the display on.
 *
 * @param display    the display instance.
 * @return           zero on success else negative error code
 *                   from esp_err_t.
 */
int32_t ht16k33a_set_display_on(display_t *display);

/** Turn the display off (blanked, contents retained).
 *
 * @param display    the display instance.
 * @return           zero on success else negative error code
 *                   from esp_err_t.
 */
int32_t ht16k33a_set_display_off(display_t *display);

/** Write a string to the display.
 *
 * The string is held internally (so may be on the stack); what is
 * actually shown depends on the scroll mode set with
 * ht16k33a_set_scroll_mode():
 *
 *  - HT16K33A_SCROLL_NONE: the first N characters are shown, where
 *    N is the total number of digits across all units; any excess is
 *    discarded.
 *  - HT16K33A_SCROLL_LEFT/RIGHT: the string is scrolled one digit at
 *    a time at the rate set with ht16k33a_set_scroll_rate().  The
 *    scroll runs in the background (from a timer/task) and this call
 *    returns immediately.
 *
 * What happens when the scroll reaches the end of the text is
 * governed by ht16k33a_set_scroll_end().
 *
 * Characters that cannot be represented on a 14-segment display
 * (e.g. lowercase letters, most punctuation) are shown as best they
 * can be or as a blank.
 *
 * If scroll mode is active, this call also resets the scroll
 * position to the start of the new string.
 *
 * @param display the display instance.
 * @param text    the null-terminated string to display; may be NULL
 *                or empty to clear the display.
 * @return        zero on success else negative error code from
 *                esp_err_t.
 */
int32_t ht16k33a_set_text(display_t *display, const char *text);

/** Write a string to the display, fixed and centred, without
 * scrolling.  Characters beyond the display width are discarded.
 * This is a convenience wrapper around
 * ht16k33a_set_scroll_mode(HT16K33A_SCROLL_NONE) plus
 * ht16k33a_set_text().
 *
 * @param display the display instance.
 * @param text    the null-terminated string to display.
 * @return        zero on success else negative error code from
 *                esp_err_t.
 */
int32_t ht16k33a_set_text_fixed(display_t *display, const char *text);

/** Set the scroll mode.
 *
 * @param display the display instance.
 * @param mode    the desired scroll mode.
 * @return        zero on success else negative error code from
 *                esp_err_t.
 */
int32_t ht16k33a_set_scroll_mode(display_t *display,
                                 ht16k33a_scroll_mode_t mode);

/** Get the scroll mode.
 *
 * @param display the display instance.
 * @return        the scroll mode, else negative error code from
 *                esp_err_t.
 */
int32_t ht16k33a_get_scroll_mode(display_t *display);

/** Set what happens when the scroll reaches the end of the text.
 *
 * The default set by ht16k33a_create() is HT16K33A_SCROLL_WRAP.
 *
 * HT16K33A_SCROLL_STOP_AT_END is the "ticker" behaviour: the text
 * scrolls on once and then holds with the tail end fully visible,
 * until the text is changed or ht16k33a_resume_scroll() is called.
 *
 * @param display the display instance.
 * @param mode    the desired end-of-scroll behaviour.
 * @return        zero on success else negative error code from
 *                esp_err_t.
 */
int32_t ht16k33a_set_scroll_end(display_t *display,
                                ht16k33a_scroll_end_t mode);

/** Get what happens when the scroll reaches the end of the text.
 *
 * @param display the display instance.
 * @return        the end-of-scroll behaviour, else negative error
 *                code from esp_err_t.
 */
int32_t ht16k33a_get_scroll_end(display_t *display);

/** Set the scroll rate.
 *
 * @param display      the display instance.
 * @param milliseconds time between one-digit scroll steps; a
 *                     value of 0 uses a sensible default
 *                     (e.g. 250 ms).
 * @return             zero on success else negative error code
 *                     from esp_err_t.
 */
int32_t ht16k33a_set_scroll_rate(display_t *display,
                                 uint32_t milliseconds);

/** Freeze the scroll engine exactly where it is.  The display keeps
 * showing the current window.  Call ht16k33a_resume_scroll() to
 * continue.  This is an *immediate* control and is independent of
 * ht16k33a_set_scroll_end(): it overrides whatever the end policy
 * would have done until resumed.
 *
 * @param display the display instance.
 * @return        zero on success else negative error code from
 *                esp_err_t.
 */
int32_t ht16k33a_pause_scroll(display_t *display);

/** Resume a scroll previously paused with ht16k33a_pause_scroll().
 *
 * @param display the display instance.
 * @return        zero on success else negative error code from
 *                esp_err_t.
 */
int32_t ht16k33a_resume_scroll(display_t *display);

/** Register a callback to be invoked each time the scroll reaches
 * its end point.  Only meaningful with HT16K33A_SCROLL_STOP_AT_END;
 * with the other end modes it is never called.  Pass NULL to clear.
 *
 * @param display   the display instance.
 * @param callback  the function to call, or NULL.
 * @param user_data opaque pointer passed to the callback.
 * @return          zero on success else negative error code from
 *                  esp_err_t.
 */
int32_t ht16k33a_set_scroll_done_callback(display_t *display,
                                          ht16k33a_scroll_done_t callback,
                                          void *user_data);

/** Directly set the text to display without any scrolling, giving
 * the caller full control (e.g. for animations).  This bypasses the
 * scroll engine and stops it until ht16k33a_set_text() is next
 * called.
 *
 * @param display the display instance.
 * @param text    the null-terminated string; the first N characters
 *                (N = total digits) are shown, remainder discarded.
 * @return        zero on success else negative error code from
 *                esp_err_t.
 */
int32_t ht16k33a_write_immediate(display_t *display, const char *text);

/** Clear the display (all digits blank) and stop scrolling.
 *
 * @param display the display instance.
 * @return        zero on success else negative error code from
 *                esp_err_t.
 */
int32_t ht16k33a_clear(display_t *display);

/** Get the total number of digits of a display.
 *
 * @param display the display instance.
 * @return        the number of digits, else negative error code
 *                from esp_err_t.
 */
int32_t ht16k33a_get_num_digits(display_t *display);

#ifdef __cplusplus
}
#endif

#endif // _HT16K33A_H_

// End of file