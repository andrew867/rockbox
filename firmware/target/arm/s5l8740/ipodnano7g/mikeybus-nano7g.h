/***************************************************************************
 * Apple MikeyBus (headset identity and inline remote) for the iPod nano 7G.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#ifndef __MIKEYBUS_NANO7G_H__
#define __MIKEYBUS_NANO7G_H__

#include <stdbool.h>
#include <stdint.h>

bool mikeybus_init(void);

/* Drain the UART and update state. Cheap; safe to call from the button tick. */
void mikeybus_poll(void);

/* True when a headset is identified as present (not open-circuit/default). */
bool mikeybus_jack_present(void);

uint8_t mikeybus_model(void);
const char *mikeybus_model_name(uint8_t model);

/* Apply a model sample, including the stock sample-15 remap. */
void mikeybus_set_model_sample(uint8_t sample, uint8_t modifier);

/*
 * Inline remote buttons, as BUTTON_* bits.
 *
 * The wire encoding behind this is inference, not RE -- see the long comment
 * in mikeybus-nano7g.c. Use mikeybus_raw_stream() to read the real codes off
 * a device with a remote attached and correct decode_remote() accordingly.
 */
int mikeybus_read_buttons(void);

/* Raw post-escape byte stream ring, for the debug screen. */
const uint8_t *mikeybus_raw_stream(unsigned *head);

void mikeybus_get_counters(unsigned *rx_bytes, unsigned *packets);

#endif /* __MIKEYBUS_NANO7G_H__ */
