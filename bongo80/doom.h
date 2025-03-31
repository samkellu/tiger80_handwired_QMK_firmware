/* Copyright 2023 Sam Kelly (@samkellu)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef USE_EMULATOR
  #include "../emulator/emulator.h"
#else
  #include "quantum.h"
#endif

#define SCREEN_WIDTH          128
#define SCREEN_HEIGHT         64
#define OLED_RESET            -1
#define SCREEN_ADDRESS        0x3C

#define PI                    3.14159
#define START_TIME_MILLI      4000
#define FRAME_TIME_MILLI      20

#define WALL_COLLISION_DIST   5
#define ENEMY_VISION_RANGE    100
#define ENEMY_WALK_SPEED      1
#define ROTATION_SPEED        5
#define WALK_SPEED            2
#define DOV                   400.0f
#define FOV                   80.0f
#define MAX_VIEW_DIST         100000.0f
#define UI_HEIGHT             54
#define WALL_OFFSET           27
#define GUN_X                 SCREEN_WIDTH / 2
#define GUN_Y                 UI_HEIGHT
#define MIN_ROOM_WIDTH        20
#define MAP_GEN_REC_DEPTH     4
#define MAP_WIDTH             300
#define MAP_HEIGHT            300
#define DOOR_WIDTH            20
#define DOOR_IDX              0

// Represents a place in 2D space
typedef struct vec2 {
  float x, y;
} vec2;

typedef enum wall_tex {
  NONE,
  CHECK,
  DOOR
} wall_tex;

// A line segment or ray
typedef struct segment {
  vec2 u, v;
  wall_tex tex;
} segment;

// Control package given as input
typedef struct controls {
  bool l;
  bool r;
  bool u;
  bool d;
  bool shoot;
} controls;

typedef struct sprite {
  const char* mask;
  const char* bmp;
  const uint16_t size;
  const uint8_t width;
  const uint8_t height;
} sprite;

typedef struct enemy {
  vec2 pos;
  int health;
  int width;
  float direction;
  int anim_state;
  const sprite* s;
  int num_sprites;
  const sprite* s_hurt;
  int num_hurt_sprites;
} enemy;

typedef struct depth_buf_info {
  float depth;
  bool phase;
  int length;
  bool is_checked;
} depth_buf_info;

#define NUM_ENEMIES 2
enemy enemies[2];

// Doom logo intro screen, stored in PROGMEM to save global section space
#define LOGO_WIDTH 128
#define LOGO_HEIGHT 64
static const char doom_logo[] PROGMEM = {
  0x7F, 0xFF, 0xFF, 0xFE, 0x03, 0xFF, 0xFF, 0xF8, 0x1F, 0xFF, 0xFF, 0xC3, 0xFF, 0x80, 0xFF, 0xFE,
  0x3F, 0xFF, 0xFF, 0xFF, 0x87, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xFF, 0xF3, 0xFF, 0x81, 0xFF, 0xFC,
  0x1F, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFE, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xC1, 0xFF, 0xF8,
  0x07, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xC1, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xC3, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xE3, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xE3, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xE7, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xF7, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xF7, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xF7, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x0F, 0xFF, 0x7F, 0xF0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x01, 0xFF, 0xDF, 0xFC, 0x07, 0xFF, 0x7F, 0xE0, 0x3F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x03, 0xFF, 0xDF, 0xFE, 0x07, 0xFF, 0x7F, 0xE0, 0x7F, 0xFB, 0xFF, 0xFF, 0xFF, 0xE0,
  0x07, 0xFF, 0x07, 0xFF, 0xDF, 0xFF, 0x07, 0xFF, 0x7F, 0xE0, 0xFF, 0xFB, 0xFF, 0xFE, 0xFF, 0xE0,
  0x07, 0xFF, 0x1F, 0xFF, 0xDF, 0xFF, 0xC7, 0xFF, 0x7F, 0xE3, 0xFF, 0xFB, 0xFF, 0xBE, 0xFF, 0xE0,
  0x07, 0xFF, 0x3F, 0xFF, 0xDF, 0xFF, 0xE7, 0xFF, 0x7F, 0xE7, 0xFF, 0xFB, 0xFF, 0xBE, 0xFF, 0xE0,
  0x07, 0xFF, 0x7F, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0xBC, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFB, 0xFF, 0x9C, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xF3, 0xFF, 0x9C, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xE7, 0xFF, 0x9C, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0xC1, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0x83, 0xFF, 0x98, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0x00, 0xFF, 0x88, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xFF, 0x7F, 0xFF, 0xFC, 0x00, 0x7F, 0x88, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xFC, 0x00, 0x1F, 0xFF, 0xFF, 0x7F, 0xFF, 0xF8, 0x00, 0x1F, 0x80, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xFC, 0x3F, 0xFF, 0xF0, 0x00, 0x0F, 0x80, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0xE0, 0x00, 0x03, 0xFF, 0xF8, 0x1F, 0xFF, 0xC0, 0x00, 0x07, 0x80, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0x80, 0x00, 0x01, 0xFF, 0xF0, 0x0F, 0xFF, 0x80, 0x00, 0x01, 0x80, 0xFF, 0xE0,
  0x07, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x7F, 0xC0, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xE0,
  0x07, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x3F, 0x80, 0x01, 0xFC, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE0,
  0x07, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE0,
  0x07, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE0,
  0x07, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE0,
  0x07, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE0,
  0x07, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE0,
  0x07, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xE0,
  0x07, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xE0,
  0x07, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xE0,
  0x07, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE0,
  0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xE0,
  0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
  0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const sprite doom_logo_sprite = {
  NULL,
  doom_logo,
  sizeof(doom_logo),
  LOGO_WIDTH,
  LOGO_HEIGHT
};

// Gun sprite and mask
#define GUN_WIDTH   32
#define GUN_HEIGHT  32
static const char gun_bmp[] PROGMEM = {
  0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xd8, 0x00, 0x00, 0x01, 0xc4, 0x00, 0x00, 0x02, 0x04, 0x00,
  0x00, 0x02, 0x02, 0x00, 0x00, 0x02, 0xea, 0x00, 0x00, 0x04, 0xd1, 0x00, 0x00, 0x09, 0x88, 0x80,
  0x00, 0x19, 0x00, 0x00, 0x00, 0x0d, 0xc2, 0x80, 0x00, 0x29, 0x81, 0xc0, 0x00, 0x0b, 0xa2, 0x20,
  0x00, 0x31, 0x40, 0x40, 0x00, 0x23, 0x00, 0xc0, 0x00, 0x13, 0x00, 0x40, 0x00, 0x72, 0x02, 0x00,
  0x00, 0x49, 0x00, 0x40, 0x01, 0xe0, 0xa8, 0x20, 0x07, 0xf1, 0x00, 0x30, 0x0b, 0xb9, 0xe0, 0xe8,
  0x07, 0x5c, 0x03, 0xfc, 0x07, 0xef, 0xff, 0xee, 0x07, 0x75, 0x7f, 0xd2, 0x1b, 0xbb, 0xff, 0xb2,
  0x11, 0x57, 0x7d, 0x64, 0x32, 0xaf, 0xff, 0xe8, 0x13, 0x5f, 0x75, 0xd0, 0x33, 0xff, 0xfb, 0x98,
  0x17, 0xd7, 0xe5, 0x00, 0x1b, 0x8f, 0xb2, 0x30, 0x03, 0x7d, 0x58, 0x10, 0x6f, 0xbf, 0xec, 0x20
};

static const char gun_bmp_mask[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x01, 0xF8, 0x00, 
  0x00, 0x01, 0xFC, 0x00, 0x00, 0x01, 0x14, 0x00, 0x00, 0x03, 0x2E, 0x00, 0x00, 0x06, 0x77, 0x00, 
  0x00, 0x07, 0xFF, 0x00, 0x00, 0x02, 0x3D, 0x00, 0x00, 0x16, 0x7F, 0x00, 0x00, 0x04, 0x5D, 0xC0, 
  0x00, 0x0E, 0xBF, 0x80, 0x00, 0x1C, 0xFF, 0x00, 0x00, 0x0C, 0xFF, 0x80, 0x00, 0x0D, 0xFF, 0x80, 
  0x00, 0x36, 0xFF, 0x80, 0x00, 0x1F, 0x57, 0xC0, 0x00, 0x0E, 0xFF, 0xC0, 0x04, 0x46, 0x1F, 0x10, 
  0x00, 0xA3, 0xFC, 0x00, 0x00, 0x10, 0x00, 0x10, 0x00, 0x8A, 0x80, 0x2C, 0x04, 0x44, 0x00, 0x4C, 
  0x0E, 0xA8, 0x82, 0x98, 0x0D, 0x50, 0x00, 0x10, 0x0C, 0xA0, 0x8A, 0x20, 0x0C, 0x00, 0x04, 0x60, 
  0x08, 0x28, 0x1B, 0xE0, 0x04, 0x70, 0x4D, 0xC0, 0x07, 0x82, 0xA7, 0xE0, 0x10, 0x40, 0x13, 0xC0
};

const sprite gun_sprite = {
  gun_bmp_mask,
  gun_bmp,
  sizeof(gun_bmp),
  GUN_WIDTH,
  GUN_HEIGHT
};

// Muzzle flash sprite
#define FLASH_WIDTH 16
#define FLASH_HEIGHT 15
static const char PROGMEM muzzle_flash_bmp [] = {
  0x01, 0x00, 0x03, 0xC0, 0x05, 0xC0, 0x07, 0xE0, 0x0F, 0xE0, 0x3F, 0xE0, 0x3F, 0xD0, 0x3F, 0xFC,
  0x3F, 0xFC, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFC, 0x2F, 0xFC
};

const sprite muzzle_flash_sprite = {
  NULL,
  muzzle_flash_bmp,
  sizeof(muzzle_flash_bmp),
  FLASH_WIDTH,
  FLASH_HEIGHT
};

// Imp sprite and mask
#define IMP_WIDTH   32
#define IMP_HEIGHT  48
static const char imp_bmp_1[] PROGMEM = {
  0x00, 0x27, 0x20, 0x00, 0x00, 0x38, 0xE0, 0x00, 0x00, 0x30, 0x60, 0x00, 0x00, 0x10, 0x40, 0x00,
  0x00, 0x2D, 0xA0, 0x00, 0x00, 0x25, 0x20, 0x00, 0x00, 0x12, 0x40, 0x00, 0x00, 0x15, 0x40, 0x00,
  0x00, 0x2D, 0xB0, 0x60, 0x30, 0x47, 0x09, 0xC0, 0x1F, 0xC2, 0x0F, 0x80, 0x0E, 0x00, 0x03, 0x00,
  0x0C, 0x00, 0x01, 0xA0, 0x04, 0x00, 0x00, 0xB0, 0x2C, 0x00, 0x00, 0xE0, 0x68, 0x02, 0x00, 0x60,
  0x38, 0x02, 0x00, 0x20, 0x30, 0x0F, 0x80, 0x20, 0x63, 0x82, 0x07, 0x20, 0x44, 0xC2, 0x0D, 0xB0,
  0x44, 0x47, 0x08, 0x90, 0x48, 0x42, 0x10, 0xD0, 0x48, 0x4F, 0x90, 0x48, 0x50, 0x42, 0x10, 0x48,
  0xD0, 0x42, 0x08, 0x48, 0xA0, 0x40, 0x08, 0x50, 0xA0, 0x40, 0x08, 0x50, 0xA0, 0xC0, 0x04, 0xD0,
  0xB0, 0x80, 0x05, 0x20, 0x90, 0x80, 0x85, 0x20, 0x50, 0x87, 0x84, 0xC0, 0x60, 0x84, 0x84, 0x00,
  0x00, 0x84, 0x84, 0x00, 0x00, 0x8C, 0x84, 0x00, 0x00, 0x88, 0x84, 0x00, 0x00, 0x88, 0x84, 0x00,
  0x00, 0x4C, 0xC8, 0x00, 0x00, 0x44, 0x48, 0x00, 0x00, 0x24, 0x48, 0x00, 0x00, 0x24, 0x48, 0x00,
  0x00, 0x32, 0x48, 0x00, 0x00, 0x32, 0x50, 0x00, 0x00, 0x12, 0x50, 0x00, 0x00, 0x1C, 0x50, 0x00,
  0x00, 0x0C, 0x58, 0x00, 0x00, 0x1C, 0x68, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00
};

static const char imp_bmp_mask_1[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x0F, 0x80, 0x00,
  0x00, 0x12, 0x40, 0x00, 0x00, 0x1A, 0xC0, 0x00, 0x00, 0x0D, 0x80, 0x00, 0x00, 0x0A, 0x80, 0x00,
  0x00, 0x12, 0x40, 0x00, 0x00, 0x38, 0xF0, 0x00, 0x00, 0x3D, 0xF0, 0x00, 0x01, 0xFF, 0xFC, 0x00,
  0x03, 0xFF, 0xFE, 0x00, 0x03, 0xFF, 0xFF, 0x00, 0x03, 0xFF, 0xFF, 0x00, 0x07, 0xFD, 0xFF, 0x80,
  0x07, 0xFD, 0xFF, 0xC0, 0x0F, 0xF0, 0x7F, 0xC0, 0x1C, 0x7D, 0xF8, 0xC0, 0x38, 0x3D, 0xF0, 0x40,
  0x38, 0x38, 0xF0, 0x60, 0x30, 0x3D, 0xE0, 0x20, 0x30, 0x30, 0x60, 0x30, 0x20, 0x3D, 0xE0, 0x30,
  0x20, 0x3D, 0xF0, 0x30, 0x40, 0x3F, 0xF0, 0x20, 0x40, 0x3F, 0xF0, 0x20, 0x40, 0x3F, 0xF8, 0x20,
  0x40, 0x7F, 0xF8, 0xC0, 0x60, 0x7F, 0x78, 0xC0, 0x20, 0x78, 0x78, 0x00, 0x00, 0x78, 0x78, 0x00,
  0x00, 0x78, 0x78, 0x00, 0x00, 0x70, 0x78, 0x00, 0x00, 0x70, 0x78, 0x00, 0x00, 0x70, 0x78, 0x00,
  0x00, 0x30, 0x30, 0x00, 0x00, 0x38, 0x30, 0x00, 0x00, 0x18, 0x30, 0x00, 0x00, 0x18, 0x30, 0x00,
  0x00, 0x0C, 0x30, 0x00, 0x00, 0x0C, 0x20, 0x00, 0x00, 0x0C, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00,
  0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00 
};

static const char imp_bmp_2[] PROGMEM = {
  0x00, 0x27, 0x20, 0x00, 0x00, 0x38, 0xE0, 0x00, 0x00, 0x30, 0x60, 0x00, 0x00, 0x10, 0x40, 0x00,
  0x00, 0x2D, 0xA0, 0x00, 0x00, 0x25, 0x20, 0x00, 0x00, 0x12, 0x40, 0x00, 0x00, 0x15, 0x40, 0x00,
  0x30, 0x6D, 0xA0, 0x00, 0x1C, 0x87, 0x10, 0x60, 0x0F, 0x82, 0x1F, 0xC0, 0x06, 0x00, 0x03, 0x80,
  0x2C, 0x00, 0x01, 0x80, 0x68, 0x00, 0x01, 0x00, 0x38, 0x00, 0x01, 0xA0, 0x30, 0x02, 0x00, 0xB0,
  0x20, 0x02, 0x00, 0xE0, 0x20, 0x0F, 0x80, 0x60, 0x27, 0x02, 0x0E, 0x30, 0x6D, 0x82, 0x19, 0x10,
  0x48, 0x87, 0x11, 0x10, 0x58, 0x42, 0x10, 0x90, 0x90, 0x4F, 0x90, 0x90, 0x90, 0x42, 0x10, 0x50,
  0x90, 0x82, 0x10, 0x58, 0x50, 0x80, 0x10, 0x28, 0x50, 0x80, 0x10, 0x28, 0x59, 0x00, 0x18, 0x28,
  0x25, 0x00, 0x08, 0x68, 0x25, 0x08, 0x08, 0x48, 0x19, 0x0F, 0x08, 0x50, 0x01, 0x09, 0x08, 0x30,
  0x01, 0x09, 0x08, 0x00, 0x01, 0x09, 0x88, 0x00, 0x01, 0x08, 0x88, 0x00, 0x01, 0x08, 0x88, 0x00,
  0x00, 0x99, 0x90, 0x00, 0x00, 0x91, 0x10, 0x00, 0x00, 0x91, 0x20, 0x00, 0x00, 0x91, 0x20, 0x00,
  0x00, 0x92, 0x60, 0x00, 0x00, 0x52, 0x60, 0x00, 0x00, 0x52, 0x40, 0x00, 0x00, 0x51, 0xC0, 0x00,
  0x00, 0xD1, 0x80, 0x00, 0x00, 0xB1, 0xC0, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0x20, 0x00
};

static const char imp_bmp_mask_2[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x0F, 0x80, 0x00,
  0x00, 0x12, 0x40, 0x00, 0x00, 0x1A, 0xC0, 0x00, 0x00, 0x0D, 0x80, 0x00, 0x00, 0x0A, 0x80, 0x00,
  0x00, 0x12, 0x40, 0x00, 0x00, 0x78, 0xE0, 0x00, 0x00, 0x7D, 0xE0, 0x00, 0x01, 0xFF, 0xFC, 0x00,
  0x03, 0xFF, 0xFE, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x0F, 0xFD, 0xFF, 0x00,
  0x1F, 0xFD, 0xFF, 0x00, 0x1F, 0xF0, 0x7F, 0x80, 0x18, 0xFD, 0xF1, 0xC0, 0x10, 0x7D, 0xE0, 0xE0,
  0x30, 0x78, 0xE0, 0xE0, 0x20, 0x3D, 0xE0, 0x60, 0x60, 0x30, 0x60, 0x60, 0x60, 0x3D, 0xE0, 0x20,
  0x60, 0x7D, 0xE0, 0x20, 0x20, 0x7F, 0xE0, 0x10, 0x20, 0x7F, 0xE0, 0x10, 0x20, 0xFF, 0xE0, 0x10,
  0x18, 0xFF, 0xF0, 0x10, 0x18, 0xF7, 0xF0, 0x30, 0x00, 0xF0, 0xF0, 0x20, 0x00, 0xF0, 0xF0, 0x00,
  0x00, 0xF0, 0xF0, 0x00, 0x00, 0xF0, 0x70, 0x00, 0x00, 0xF0, 0x70, 0x00, 0x00, 0xF0, 0x70, 0x00,
  0x00, 0x60, 0x60, 0x00, 0x00, 0x60, 0xE0, 0x00, 0x00, 0x60, 0xC0, 0x00, 0x00, 0x60, 0xC0, 0x00,
  0x00, 0x61, 0x80, 0x00, 0x00, 0x21, 0x80, 0x00, 0x00, 0x21, 0x80, 0x00, 0x00, 0x20, 0x00, 0x00,
  0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0xC0, 0x00
};

const sprite imp_sprite_1 = {
  imp_bmp_mask_1,
  imp_bmp_1,
  sizeof(imp_bmp_1),
  IMP_WIDTH,
  IMP_HEIGHT
};

const sprite imp_sprite_2 = {
  imp_bmp_mask_2,
  imp_bmp_2,
  sizeof(imp_bmp_2),
  IMP_WIDTH,
  IMP_HEIGHT
};

const sprite imp_sheet[] = {imp_sprite_1, imp_sprite_2};

const sprite imp_sprite_hurt_1 = {
  imp_bmp_1,
  imp_bmp_mask_1,
  sizeof(imp_bmp_1),
  IMP_WIDTH,
  IMP_HEIGHT
};

const sprite imp_sprite_hurt_2 = {
  imp_bmp_2,
  imp_bmp_mask_2,
  sizeof(imp_bmp_2),
  IMP_WIDTH,
  IMP_HEIGHT
};

const sprite imp_hurt_sheet[] = {imp_sprite_hurt_1, imp_sprite_hurt_2};

float pow2(float x);

float dot(vec2 v, vec2 u);

float dist2(vec2 v, vec2 u);

vec2 sub(vec2 v, vec2 u);

vec2 add(vec2 v, vec2 u);

vec2 proj(vec2 v, vec2 u);

float inv_sqrt(float num);

void doom_setup(void);

void draw_gun(bool moving, bool show_flash);

void doom_update(controls c);

void render_map(vec2 p, int pa, bool is_shooting);

vec2 raycast(segment s1, segment s2, bool* hit);

void vertical_line(int x, int half_length, bool color, int skip);

void check_line(int x, int half_length, bool phase);

void enemy_update(void);

void reload_enemy(enemy* e);

bool collision_detection(vec2 p, bool is_enemy);

float point_ray_dist2(vec2 p, segment s);

void oled_write_bmp_P(const sprite img, int x, int y);

void oled_write_bmp_P_scaled(sprite img, int draw_height, int draw_width, int x, int y);

void doom_dispose(void);

vec2 get_valid_spawn(void);

const char* get_u32_str(uint32_t value);

segment* bsp_wallgen(segment* walls, int* num_walls, int l, int r, int t, int b, int depth);