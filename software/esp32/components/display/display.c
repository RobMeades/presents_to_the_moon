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
 * @brief Implementation of a time-aware display manager.
 *
 * The manager owns one ht16k33a_display_t handle per display_t and
 * drives it from a single fgr_task, polling registered sources and
 * arbitrating between them and any active transient message.
 */

#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "sys/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "fgr_util.h"
#include "fgr_task.h"
#include "ht16k33a.h"
#include "display.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Logging prefix
#define TAG "display"

// Manager task stack.
#define DISPLAY_TASK_STACK_BYTES 4096

// Display priority (low).
#define DISPLAY_TASK_PRIORITY    2

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// One registered text source.
struct display_source {
    display_t           *display;      // owning display; NULL once destroyed
    display_source_cb_t  cb;
    void                *user_data;
    int32_t              priority;     // lower = higher priority
    uint32_t             refresh_ms;
    const char          *name;
    int64_t              next_due_us;  // esp_timer_get_time() target
    char                 last_text[DISPLAY_SOURCE_TEXT_LENGTH];
    SLIST_ENTRY(display_source) next;
};

// A logical display.
struct display {
    ht16k33a_display_t  *hw;           // underlying ht16k33a handle

    // Transient message (overrides sources while active).
    char                 message[DISPLAY_MESSAGE_TEXT_LENGTH];
    int64_t              message_until_us;  // 0 = no active message
    bool                 message_active;

    // Sources, kept in registration order; sorted at use time.
    SLIST_HEAD(source_list_t, display_source) sources;
    size_t               num_sources;

    // Last text handed to the underlying driver, so we don't
    // hammer the I2C bus with identical updates every loop.
    char                 shown[DISPLAY_MESSAGE_TEXT_LENGTH];

    SLIST_ENTRY(display) next;
};

SLIST_HEAD(display_list_t, display);

typedef struct {
    SemaphoreHandle_t lock;
    bool              initialised;
    bool              task_running;
    void             *task_handle;
    int32_t           i2c_port;
    int32_t           sda_pin;
    int32_t           scl_pin;
    int32_t           clock_hz;
    struct display_list_t display_list;
} context_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

static context_t g_context = {0};

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Poll a single source if it is due, updating its cached text and
// its next-due time.  Returns true if the source has non-empty
// text to show after the poll (whether or not it was due this
// time), false otherwise.  The source's text is taken to be
// whatever is currently cached in last_text[], so a source that
// was polled on a previous pass and returned text continues to
// "win" on passes where it isn't polled.
// IMPORTANT: the context must be locked before this is called.
static bool source_poll(display_source_t *source)
{
    int64_t now = esp_timer_get_time();

    if (now >= source->next_due_us) {
        char buffer[DISPLAY_SOURCE_TEXT_LENGTH];
        memset(buffer, 0, sizeof(buffer));

        if (source->cb(buffer, sizeof(buffer), source->user_data)) {
            strncpy(source->last_text, buffer,
                    sizeof(source->last_text) - 1);
            source->last_text[sizeof(source->last_text) - 1] = '\0';
        } else {
            source->last_text[0] = '\0';
        }

        // Schedule the next poll; use a fresh `now` so a long
        // callback doesn't make the next poll immediately due.
        source->next_due_us = esp_timer_get_time() +
                              (int64_t) source->refresh_ms * 1000;
    }

    return source->last_text[0] != '\0';
}

// Walk the display's sources in priority order (they are kept
// sorted by display_source_add() and display_source_set_priority())
// and copy the highest-priority non-empty text into candidate.
// Every source that is due is polled, so each source's refresh
// rate is honoured regardless of what higher-priority sources
// are doing.
// IMPORTANT: the context must be locked before this is called.
static void pick_source_text(display_t *display,
                             char *candidate, size_t candidate_size)
{
    display_source_t *iter;

    SLIST_FOREACH(iter, &display->sources, next) {
        if (source_poll(iter) && (candidate[0] == '\0')) {
            // First non-empty source in priority order wins.
            strncpy(candidate, iter->last_text, candidate_size - 1);
            candidate[candidate_size - 1] = '\0';
        }
    }
}

// Render a display's current state (message or best source) to
// the underlying ht16k33a handle, if the text has changed.
// IMPORTANT: the context must be locked before this is called.
static int32_t render_current(display_t *display)
{
    int32_t err = ESP_OK;
    char candidate[DISPLAY_MESSAGE_TEXT_LENGTH];

    candidate[0] = '\0';

    if (display->message_active) {
        // Transient message wins.
        strncpy(candidate, display->message, sizeof(candidate) - 1);
        candidate[sizeof(candidate) - 1] = '\0';
    } else {
        pick_source_text(display, candidate, sizeof(candidate));
    }

    // Only push to the hardware if it differs from what we last sent.
    if (strcmp(candidate, display->shown) != 0) {
        strncpy(display->shown, candidate, sizeof(display->shown) - 1);
        display->shown[sizeof(display->shown) - 1] = '\0';
        err = ht16k33a_set_text(display->hw, candidate);
    }

    return err;
}

// The manager task callback, called in a loop by fgr_task.
static void display_task_cb(void *handle, void *param)
{
    (void) handle;
    (void) param;

    CONTEXT_LOCK(g_context.lock, "display_task_cb()");

    display_t *display;
    SLIST_FOREACH(display, &g_context.display_list, next) {
        // Expire any transient message whose time is up.
        if (display->message_active) {
            int64_t now = esp_timer_get_time();
            if ((display->message_until_us > 0) &&
                (now >= display->message_until_us)) {
                display->message_active = false;
                display->message_until_us = 0;
                display->message[0] = '\0';
            }
        }

        int32_t err = render_current(display);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "render failed %s (%d).", esp_err_to_name(-err), err);
        }
    }

    CONTEXT_UNLOCK(g_context.lock, "display_task_cb()");

    vTaskDelay(pdMS_TO_TICKS(50));
}

// Apply a scroll configuration to the underlying ht16k33a handle.
// IMPORTANT: the context must be locked before this is called.
static void apply_scroll(display_t *display,
                         ht16k33a_scroll_mode_t mode,
                         ht16k33a_scroll_end_t end,
                         uint32_t rate_ms)
{
    ht16k33a_set_scroll_mode(display->hw, mode);
    ht16k33a_set_scroll_end(display->hw, end);
    ht16k33a_set_scroll_rate(display->hw, rate_ms);
}

// Clean up a single display.
// IMPORTANT: the context must be locked before this is called.
static void clean_up(display_t *display)
{
    if (display) {
        // Destroy sources first: they hold pointers back to display.
        while (!SLIST_EMPTY(&display->sources)) {
            display_source_t *source = SLIST_FIRST(&display->sources);
            SLIST_REMOVE_HEAD(&display->sources, next);
            display->num_sources--;
            free(source);
        }

        if (display->hw) {
            ht16k33a_destroy(display->hw);
            display->hw = NULL;
        }

        // Remove from the global list.
        display_t *iter;
        display_t *prev = NULL;
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
 * PUBLIC FUNCTIONS: LIFECYCLE
 * -------------------------------------------------------------- */

// Initialise the display management task and underlying HT16K33A.
int32_t display_init(int32_t i2c_port, int32_t sda_pin,
                     int32_t scl_pin, int32_t clock_hz)
{
    int32_t err = ESP_OK;

    if (!g_context.lock) {
        err = -ESP_ERR_NO_MEM;
        g_context.lock = xSemaphoreCreateMutex();
        if (g_context.lock != NULL) {
            err = ESP_OK;
        }
        SLIST_INIT(&g_context.display_list);
    }

    if (err == ESP_OK) {

        CONTEXT_LOCK(g_context.lock, "display_init()");

        if (!g_context.initialised) {
            err = ht16k33a_init(i2c_port, sda_pin, scl_pin, clock_hz);

            if (err == ESP_OK) {
                g_context.i2c_port = i2c_port;
                g_context.sda_pin = sda_pin;
                g_context.scl_pin = scl_pin;
                g_context.clock_hz = clock_hz;

                // Create the manager task.  fgr_task_create() wants a
                // place to store the handle; the handle is a plain
                // FreeRTOS TaskHandle.
                err = fgr_task_create(display_task_cb,
                                      NULL,
                                      "display",
                                      DISPLAY_TASK_STACK_BYTES,
                                      DISPLAY_TASK_PRIORITY,
                                      &g_context.task_handle);
                if (err >= 0) {
                    g_context.task_running = true;
                    g_context.initialised = true;
                } else {
                    ESP_LOGE(TAG, "failed to create manager task, %s (%d).", esp_err_to_name(-err), err);
                    ht16k33a_deinit();
                }
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "display_init()");
    }

    return err;
}

// Deinitialise all displays.
void display_deinit()
{
    if (g_context.lock) {
        CONTEXT_LOCK(g_context.lock, "display_deinit()");

        if (g_context.initialised) {
            // Stop the manager task first so it can't touch the
            // displays while we tear them down.
            if (g_context.task_running) {
                fgr_task_destroy(g_context.task_handle);
                g_context.task_handle = NULL;
                g_context.task_running = false;
            }

            while (!SLIST_EMPTY(&g_context.display_list)) {
                display_t *display = SLIST_FIRST(&g_context.display_list);
                clean_up(display);
            }

            ht16k33a_deinit();

            g_context.initialised = false;
        }

        CONTEXT_UNLOCK(g_context.lock, "display_deinit()");
    }
}

// Create a display
display_t *display_create(const uint8_t *address_list,
                          size_t num_units,
                          ht16k33a_scroll_mode_t mode,
                          ht16k33a_scroll_end_t end,
                          uint32_t rate_ms)
{
    display_t *display = NULL;

    if (g_context.lock && g_context.initialised &&
        address_list && (num_units > 0) &&
        (num_units <= HT16K33A_MAX_UNITS)) {

        CONTEXT_LOCK(g_context.lock, "display_create()");

        display = malloc(sizeof(*display));
        if (display) {
            memset(display, 0, sizeof(*display));
            SLIST_INIT(&display->sources);

            display->hw = ht16k33a_create(address_list, num_units);
            if (display->hw) {
                // Configure scroll before any text is shown.
                apply_scroll(display, mode, end, rate_ms);
                SLIST_INSERT_HEAD(&g_context.display_list, display, next);
            } else {
                ESP_LOGE(TAG, "ht16k33a_create() failed.");
                free(display);
                display = NULL;
            }
        } else {
            ESP_LOGE(TAG, "out of memory.");
        }

        CONTEXT_UNLOCK(g_context.lock, "display_create()");
    }

    return display;
}

// Destroy a single display.
void display_destroy(display_t *display)
{
    if (g_context.lock) {
        CONTEXT_LOCK(g_context.lock, "display_destroy()");
        clean_up(display);
        CONTEXT_UNLOCK(g_context.lock, "display_destroy()");
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: CONFIGURATION
 * -------------------------------------------------------------- */

// Set the scrolling style of a display.
int32_t display_set_scroll(display_t *display,
                           ht16k33a_scroll_mode_t mode,
                           ht16k33a_scroll_end_t end,
                           uint32_t rate_ms)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.lock && g_context.initialised) {

        CONTEXT_LOCK(g_context.lock, "display_set_scroll()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            apply_scroll(display, mode, end, rate_ms);
            err = ESP_OK;
        }

        CONTEXT_UNLOCK(g_context.lock, "display_set_scroll()");
    }

    return err;
}

// Set the brightness of a display.
int32_t display_set_brightness(display_t *display,
                               ht16k33a_brightness_t brightness)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.lock && g_context.initialised) {

        CONTEXT_LOCK(g_context.lock, "display_set_brightness()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            err = ht16k33a_set_brightness(display->hw, brightness);
        }

        CONTEXT_UNLOCK(g_context.lock, "display_set_brightness()");
    }

    return err;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: TRANSIENT MESSAGES
 * -------------------------------------------------------------- */

// Display a transient message.
int32_t display_set_message(display_t *display,
                            const char *text,
                            uint32_t duration_ms)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.lock && g_context.initialised) {

        CONTEXT_LOCK(g_context.lock, "display_set_message()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            if ((text == NULL) || (text[0] == '\0')) {
                // Clearing via set_message(NULL/empty) is equivalent
                // to display_clear_message().
                display->message_active = false;
                display->message_until_us = 0;
                display->message[0] = '\0';
            } else {
                if (duration_ms == 0) {
                    duration_ms = DISPLAY_MESSAGE_DEFAULT_MS;
                }
                strncpy(display->message, text,
                        sizeof(display->message) - 1);
                display->message[sizeof(display->message) - 1] = '\0';
                display->message_active = true;
                display->message_until_us = esp_timer_get_time() +
                                            (int64_t) duration_ms * 1000;
            }
            err = ESP_OK;
        }

        CONTEXT_UNLOCK(g_context.lock, "display_set_message()");
    }

    return err;
}

// Clear a transient message.
int32_t display_clear_message(display_t *display)
{
    return display_set_message(display, NULL, 0);
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: SOURCES
 * -------------------------------------------------------------- */

// Register a source of stuff to display.
display_source_t *display_source_add(display_t *display,
                                     display_source_cb_t cb,
                                     void *user_data,
                                     int32_t priority,
                                     uint32_t refresh_ms,
                                     const char *name)
{
    display_source_t *source = NULL;

    if (g_context.lock && g_context.initialised) {

        CONTEXT_LOCK(g_context.lock, "display_source_add()");

        if (display && cb &&
            (display->num_sources < DISPLAY_MAX_SOURCES)) {
            source = malloc(sizeof(*source));
            if (source) {
                memset(source, 0, sizeof(*source));
                source->display = display;
                source->cb = cb;
                source->user_data = user_data;
                source->priority = priority;
                source->refresh_ms = (refresh_ms > 0) ? refresh_ms : DISPLAY_SOURCE_DEFAULT_REFRESH_MS;
                source->name = name;
                // Due immediately so the first text appears promptly.
                source->next_due_us = esp_timer_get_time();
                source->last_text[0] = '\0';

                // Insert in priority order; ties keep registration
                // order, so a source added later with the same
                // priority comes after one added earlier.  The list
                // is kept sorted so pick_source_text() can simply
                // walk it front-to-back.
                display_source_t *prev = NULL;
                display_source_t *iter;
                SLIST_FOREACH(iter, &display->sources, next) {
                    if (iter->priority > source->priority) {
                        break;
                    }
                    prev = iter;
                }
                if (prev == NULL) {
                    SLIST_INSERT_HEAD(&display->sources, source, next);
                } else {
                    SLIST_INSERT_AFTER(prev, source, next);
                }
                display->num_sources++;

                ESP_LOGI(TAG, "source '%s' added (priority %d, refresh %d ms).",
                         name ? name : "(unnamed)", priority,
                         source->refresh_ms);
            } else {
                ESP_LOGE(TAG, "out of memory.");
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "display_source_add()");
    }

    return source;
}

// Destroy a source of stuff to display.
void display_source_destroy(display_source_t *source)
{
    if (g_context.lock && source) {

        CONTEXT_LOCK(g_context.lock, "display_source_destroy()");

        display_t *display = source->display;
        if (display) {
            // Remove from the display's list (if still there).
            display_source_t *iter;
            display_source_t *prev = NULL;
            SLIST_FOREACH(iter, &display->sources, next) {
                if (iter == source) {
                    if (prev == NULL) {
                        SLIST_REMOVE_HEAD(&display->sources, next);
                    } else {
                        SLIST_REMOVE_AFTER(prev, next);
                    }
                    display->num_sources--;
                    break;
                }
                prev = iter;
            }
            // Mark orphaned so a render_current() in flight (which
            // releases the lock around the callback) knows to skip
            // this source if it comes back to it.
            source->display = NULL;
        }

        free(source);

        CONTEXT_UNLOCK(g_context.lock, "display_source_destroy()");
    }
}

// Modify the refresh rate of a source of display.
int32_t display_source_set_refresh(display_source_t *source,
                                   uint32_t refresh_ms)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.lock && g_context.initialised) {

        CONTEXT_LOCK(g_context.lock, "display_source_set_refresh()");

        err = -ESP_ERR_INVALID_ARG;
        if (source) {
            source->refresh_ms = (refresh_ms > 0) ?
                                 refresh_ms :
                                 DISPLAY_SOURCE_DEFAULT_REFRESH_MS;
            // Consider it due immediately after a rate change.
            source->next_due_us = esp_timer_get_time();
            err = ESP_OK;
        }

        CONTEXT_UNLOCK(g_context.lock, "display_source_set_refresh()");
    }

    return err;
}

int32_t display_source_set_priority(display_source_t *source,
                                    int32_t priority)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.lock && g_context.initialised) {

        CONTEXT_LOCK(g_context.lock, "display_source_set_priority()");

        err = -ESP_ERR_INVALID_ARG;
        if (source && source->display) {
            display_t *display = source->display;

            // Remove from its current position.
            display_source_t *prev = NULL;
            display_source_t *iter;
            SLIST_FOREACH(iter, &display->sources, next) {
                if (iter == source) {
                    if (prev == NULL) {
                        SLIST_REMOVE_HEAD(&display->sources, next);
                    } else {
                        SLIST_REMOVE_AFTER(prev, next);
                    }
                    break;
                }
                prev = iter;
            }

            // Re-insert in the new position.
            source->priority = priority;
            prev = NULL;
            SLIST_FOREACH(iter, &display->sources, next) {
                if (iter->priority > source->priority) {
                    break;
                }
                prev = iter;
            }
            if (prev == NULL) {
                SLIST_INSERT_HEAD(&display->sources, source, next);
            } else {
                SLIST_INSERT_AFTER(prev, source, next);
            }

            err = ESP_OK;
        }

        CONTEXT_UNLOCK(g_context.lock, "display_source_set_priority()");
    }

    return err;
}

// Get the name of a source of display.
const char *display_source_get_name(display_source_t *source)
{
    return source ? source->name : NULL;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: INTROSPECTION
 * -------------------------------------------------------------- */

// Get the number of digits in the display.
int32_t display_get_num_digits(display_t *display)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.lock && g_context.initialised) {

        CONTEXT_LOCK(g_context.lock, "display_get_num_digits()");

        err = -ESP_ERR_INVALID_ARG;
        if (display) {
            err = ht16k33a_get_num_digits(display->hw);
        }

        CONTEXT_UNLOCK(g_context.lock, "display_get_num_digits()");
    }

    return err;
}

// Get the text currently displayed.
int32_t display_get_current_text(display_t *display,
                                 char *buffer,
                                 size_t buffer_size)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (buffer && (buffer_size > 0)) {
        buffer[0] = '\0';
    }

    if (g_context.lock && g_context.initialised) {

        CONTEXT_LOCK(g_context.lock, "display_get_current_text()");

        err = -ESP_ERR_INVALID_ARG;
        if (display && buffer && (buffer_size > 0)) {
            strncpy(buffer, display->shown, buffer_size - 1);
            buffer[buffer_size - 1] = '\0';
            err = ESP_OK;
        }

        CONTEXT_UNLOCK(g_context.lock, "display_get_current_text()");
    }

    return err;
}

// End of file