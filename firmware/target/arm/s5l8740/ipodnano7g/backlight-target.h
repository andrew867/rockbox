#ifndef BACKLIGHT_TARGET_H
#define BACKLIGHT_TARGET_H

#include <stdbool.h>

bool backlight_hw_init(void);
void backlight_hw_on(void);
void backlight_hw_off(void);
void backlight_hw_brightness(int brightness);

#endif
