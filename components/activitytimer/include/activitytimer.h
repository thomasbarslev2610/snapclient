#ifndef ACTIVITYTIMER_H
#define ACTIVITYTIMER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * Initialise relay GPIO and RGB LED; read settings from NVS.
 * Call once after settings_manager_init().
 */
esp_err_t activitytimer_init(void);

/**
 * Notify the timer that the player started or stopped.
 * When playing becomes true  → relay HIGH, LED green immediately.
 * When playing becomes false → start the inactivity countdown;
 *                              relay LOW + LED red after timeout.
 */
void activitytimer_notify_playing(bool playing);

#endif /* ACTIVITYTIMER_H */
