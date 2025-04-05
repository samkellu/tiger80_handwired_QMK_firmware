#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// #include "../bongo80/doom.h"
// #include "../bongo80/bongo80.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define PSTR(a) (a)

#define PROGMEM
#define QK_KB_0 1
#define keyrecord_t

typedef struct led_t {
    int caps_lock;
} led_t;

static led_t led_usb_state = {1};
// static uint8_t curr_wpm = 0;


int oled_write_pixel(int, int, bool);
int timer_read();
int timer_elapsed();
uint32_t timer_elapsed32();
int oled_set_cursor(int, int);
int oled_write(const char*, int);
int oled_write_P(const char*, int);
int oled_write_raw_P(const char*, size_t);
int oled_clear();
int get_current_wpm();
int host_keyboard_led_state();
char* get_u8_str(uint8_t, char);
char* get_u16_str(uint16_t, char);
uint8_t pgm_read_byte(const void*);