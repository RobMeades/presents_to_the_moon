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

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

/** @file
 * @brief A time-aware display manager built on top of the
 * HT16K33A component: shows short "transient" messages that
 * override a set of registered text sources, each of which is
 * polled periodically.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "ht16k33a.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/** The maximum number of sources that may be registered with
 * a single display.  A source is added with display_source_add().
 */
#define DISPLAY_MAX_SOURCES 8

/** The maximum size, including the null terminator, of the
 * text buffer that a source callback must fill in.  Anything
 * longer is truncated by the display manager before being
 * handed to ht16k33a.
 */
#define DISPLAY_SOURCE_TEXT_LENGTH 128

/** The maximum size, including the null terminator, of the
 * transient message buffer.  Longer messages are truncated
 * with a warning.
 */
#define DISPLAY_MESSAGE_TEXT_LENGTH 128

/** The default duration, in milliseconds, of a transient
 * message if zero is passed to display_set_message().  Zero
 * duration means "until replaced by another message or
 * cleared with display_clear_message()".
 */
#define DISPLAY_MESSAGE_DEFAULT_MS 2000

/** The default refresh period, in milliseconds, for a source
 * that passes zero for refresh_ms when being added.  Keep this
 * comfortably above fgr_task's loop delay
 * (FGR_UTIL_WATCHDOG_FEED_TIME_MS); 250 ms is a sensible
 * default.
 */
#define DISPLAY_SOURCE_DEFAULT_REFRESH_MS 250

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* Display handle: an opaque pointer as far as the user is
 * concerned.  Do not dereference.
 */
typedef struct display display_t;

/* Source handle: an opaque pointer as far as the user is
 * concerned.  Do not dereference.  Created by
 * display_source_add() and destroyed by display_source_destroy().
 */
typedef struct display_source display_source_t;

/** A source callback: fills buffer with the text that this
 * source currently wants shown.  Return true if buffer has been
 * populated, false if the source has nothing to say right now
 * (in which case buffer's contents are ignored).  The callback
 * is invoked from the display manager's task, so it must not
 * block and it must not call back into this API.
 *
 * @param buffer      the buffer to fill; will be at most
 *                    DISPLAY_SOURCE_TEXT_LENGTH bytes including
 *                    the null terminator.  The manager pre-clears
 *                    it to an empty string.
 * @param buffer_size size of buffer in bytes.
 * @param user_data   the user_data passed to display_source_add().
 * @return            true if buffer has been filled with text to
 *                    display, false otherwise.
 */
typedef bool (*display_source_cb_t) (char *buffer,
                                     size_t buffer_size,
                                     void *user_data);

/* ----------------------------------------------------------------
 * FUNCTIONS: LIFECYCLE
 * -------------------------------------------------------------- */

/** Initialise the display manager.  This must be called before
 * any other function in this API, fgr_task_init() must have already
 * been called (as tasking is required).  This function will call
 * ht16k33a_init() on your behalf. Safe to call more than once;
 * subsequent calls do nothing and return ESP_OK.
 *
 * @param i2c_port  I2C port, e.g. I2C_NUM_0.
 * @param sda_pin   GPIO for I2C SDA.
 * @param scl_pin   GPIO for I2C SCL.
 * @param clock_hz  I2C clock frequency in Hz (400000 is typical).
 * @return          ESP_OK on success, else a negative value from
 *                  esp_err_t.
 */
int32_t display_init(int32_t i2c_port, int32_t sda_pin,
                     int32_t scl_pin, int32_t clock_hz);

/** Deinitialise the display manager.  Destroys all displays
 * created by display_create(), stops the manager task, and
 * calls ht16k33a_deinit().  It is always safe to call this,
 * even if display_init() was never called.
 */
void display_deinit();

/** Create a display instance covering one or more daisy-chained
 * HT16K33A units.
 *
 * The scroll configuration given here is applied to the
 * underlying ht16k33a handle before any text is shown and
 * remains in force until display_set_scroll() is called.
 *
 * @param address_list array of I2C addresses, one per unit, in
 *                     left-to-right display order; must contain
 *                     at least one entry.
 * @param num_units    number of units, 1 to HT16K33A_MAX_UNITS.
 * @param mode         scroll mode; HT16K33A_SCROLL_NONE
 *                     shows only the first N characters.
 * @param end          what happens at the end of the
 *                     text; only meaningful if mode is
 *                     not HT16K33A_SCROLL_NONE.
 * @param rate_ms      time between scroll steps in
 *                     milliseconds; zero uses the
 *                     ht16k33a default.
 * @return             the display instance, NULL on error.
 */
display_t *display_create(const uint8_t *address_list,
                          size_t num_units,
                          ht16k33a_scroll_mode_t mode,
                          ht16k33a_scroll_end_t end,
                          uint32_t rate_ms);

/** Destroy a display instance.  Any sources that were added to
 * this display are destroyed first (any outstanding
 * display_source_t pointers become invalid).  Any transient
 * message is discarded.
 *
 * @param display the display instance.
 */
void display_destroy(display_t *display);

/* ----------------------------------------------------------------
 * FUNCTIONS: CONFIGURATION
 * -------------------------------------------------------------- */

/** Change the scroll configuration of an existing display.  This
 * applies the new settings to the underlying ht16k33a handle
 * and takes effect from the next text update.
 *
 * @param display the display instance.
 * @param mode    scroll mode; HT16K33A_SCROLL_NONE
 *                shows only the first N characters.
 * @param end     what happens at the end of the
 *                text; only meaningful if mode is
 *                not HT16K33A_SCROLL_NONE.
 * @param rate_ms time between scroll steps in
 *                milliseconds; zero uses the
 *                ht16k33a default.
 * @return        ESP_OK on success, else a negative value from
 *                esp_err_t.
 */
int32_t display_set_scroll(display_t *display,
                           ht16k33a_scroll_mode_t mode,
                           ht16k33a_scroll_end_t  end,
                           uint32_t rate_ms);

/** Set the brightness of the display, 0 (off) to 15 (maximum).
 *
 * @param display    the display instance.
 * @param brightness the desired brightness.
 * @return           ESP_OK on success, else a negative value from
 *                   esp_err_t.
 */
int32_t display_set_brightness(display_t *display,
                               ht16k33a_brightness_t brightness);

/* ----------------------------------------------------------------
 * FUNCTIONS: TRANSIENT MESSAGES (HIGHEST PRIORITY)
 * -------------------------------------------------------------- */

/** Show a transient message immediately, overriding any active
 * source for the given duration.  If another transient message is
 * already showing, it is replaced.  The duration counts from the
 * moment this function returns.  A duration of zero means
 * DISPLAY_MESSAGE_DEFAULT_MS; to show a message with no expiry,
 * use a very large value (e.g. UINT32_MAX / 2) or call
 * display_clear_message() to remove it.
 *
 * @param display    the display instance.
 * @param text       the null-terminated message; may be NULL or
 *                   empty to clear the current message (equivalent
 *                   to display_clear_message()).
 * @param duration_ms how long to show it for, in milliseconds.
 * @return           ESP_OK on success, else a negative value from
 *                   esp_err_t.
 */
int32_t display_set_message(display_t *display,
                            const char *text,
                            uint32_t duration_ms);

/** Remove the current transient message, if any, immediately.
 * The display manager will fall back to the highest-priority
 * active source on its next loop.
 *
 * @param display the display instance.
 * @return        ESP_OK on success, else a negative value from
 *                esp_err_t.
 */
int32_t display_clear_message(display_t *display);

/* ----------------------------------------------------------------
 * FUNCTIONS: SOURCES (LOWER PRIORITY, TIME-MULTIPLEXED)
 * -------------------------------------------------------------- */

/** Register a source callback.  Sources are evaluated in priority
 * order (lower value = higher priority; ties broken by registration
 * order, earlier registration wins).  Each source is polled at most
 * once per refresh_ms; the manager keeps the last text it received
 * from a source between polls, so a source that has nothing new to
 * say does not need to update its buffer every time.
 *
 * The returned handle may be passed to display_source_destroy() at
 * any time; sources may be created and destroyed in any order.
 *
 * @param display     the display instance.
 * @param cb          the callback; cannot be NULL.
 * @param user_data   opaque pointer passed to cb; may be NULL.
 * @param priority    priority of this source: lower = higher.
 * @param refresh_ms  how often the manager should call cb; zero
 *                    uses DISPLAY_SOURCE_DEFAULT_REFRESH_MS.
 * @param name        a null-terminated name purely for debugging;
 *                    may be NULL.  If non-NULL this MUST be a true
 *                    constant valid for the lifetime of the source.
 * @return            the source handle, NULL on error.
 */
display_source_t *display_source_add(display_t *display,
                                     display_source_cb_t cb,
                                     void *user_data,
                                     int32_t priority,
                                     uint32_t refresh_ms,
                                     const char *name);

/** Destroy a source.  Safe to call from any context other than
 * from inside that source's own callback, or from inside another
 * source's callback on the same display.  If the source being
 * destroyed was the active one, the manager falls back to the
 * next-highest priority source (or blanks the display if none
 * remains) on its next loop.
 *
 * @param source the source handle, as returned by
 *               display_source_add().
 */
void display_source_destroy(display_source_t *source);

/** Change the refresh period of a source.  The new period takes
 * effect from the next poll.  The first poll after this call is
 * not delayed by the new period (i.e. the source is considered
 * "due now").
 *
 * @param source     the source handle.
 * @param refresh_ms the new refresh period in milliseconds; zero
 *                   uses DISPLAY_SOURCE_DEFAULT_REFRESH_MS.
 * @return           ESP_OK on success, else a negative value from
 *                   esp_err_t.
 */
int32_t display_source_set_refresh(display_source_t *source,
                                   uint32_t refresh_ms);

/** Change the priority of a source.  Takes effect on the next
 * manager loop.
 *
 * @param source   the source handle.
 * @param priority the new priority; lower = higher.
 * @return         ESP_OK on success, else a negative value from
 *                 esp_err_t.
 */
int32_t display_source_set_priority(display_source_t *source,
                                    int32_t priority);

/** Get the name of a source (the pointer passed to
 * display_source_add(), not a copy).
 *
 * @param source the source handle.
 * @return       the name, or NULL.
 */
const char *display_source_get_name(display_source_t *source);

/* ----------------------------------------------------------------
 * FUNCTIONS: INTROSPECTION
 * -------------------------------------------------------------- */

/** Get the number of digits of the underlying ht16k33a display.
 *
 * @param display the display instance.
 * @return        the number of digits, else a negative value from
 *                esp_err_t.
 */
int32_t display_get_num_digits(display_t *display);

/** Get the text currently being shown, if any.  This is a copy
 * of the last string handed to the underlying ht16k33a handle,
 * useful for debugging.  The buffer is pre-cleared to an empty
 * string; if the display is blank or unknown, the buffer is left
 * as an empty string and the function returns ESP_OK.
 *
 * @param display     the display instance.
 * @param buffer      the buffer to fill.
 * @param buffer_size size of buffer in bytes.
 * @return            ESP_OK on success, else a negative value from
 *                    esp_err_t.
 */
int32_t display_get_current_text(display_t *display,
                                 char *buffer,
                                 size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // _DISPLAY_H_

// End of file
