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

#include "bongo80.h"


// =================== DOOM =================== //

#include "doom.h"
#include "math.h"

// Player location and direction of camera
vec2 p;
int pa;
int shot_timer;
uint32_t time;
uint32_t game_time;
uint32_t frame_time;
uint8_t score;
int gun_x = GUN_X;
int gun_y = GUN_Y + 10;

float pow2(float x) { return x * x; }

float dot(vec2 u, vec2 v) { return u.x * v.x + u.y * v.y; }

float dist2(vec2 u, vec2 v) { return dot(sub(u, v), sub(u, v)); }

vec2 sub(vec2 u, vec2 v) { return (vec2) {u.x - v.x, u.y - v.y}; }

vec2 add(vec2 u, vec2 v) { return (vec2) {u.x + v.x, u.y + v.y}; }

vec2 proj(vec2 u, vec2 v) { 

    float t = dot(u, v) / dot(v, v);
    return (vec2) {t * v.x, t * v.y};
}

void doom_setup(void) {

    // Runs intro sequence
    oled_write_bmp_P(doom_logo, doom_logo_size, LOGO_WIDTH, LOGO_HEIGHT, 0, 0, false);
    game_time = timer_read();

    // Initializes player state
    p = (vec2) {20.0f, 20.0f};
    pa = 0;
    shot_timer = 0;
    score = 0;

    /*
    display.setTextSize(1);
    display.setTextColor(WHITE);
    */
}

void doom_update(controls c) {

    if (timer_elapsed(game_time) < START_TIME_MILLI) return;

    oled_clear();
    if (c.shoot && shot_timer == 0) {
        shot_timer = 5;
    }

    if (shot_timer > 0) {
        shot_timer--;
    }

    if (c.l) {
        pa = (pa - rotSpeed < 0 ? pa - rotSpeed + 360 : pa - rotSpeed);
    }

    if (c.r) {
        pa = (pa + rotSpeed >= 360 ? pa + rotSpeed - 360 : pa + rotSpeed);
    }

    if (c.f) {
        vec2 pn = {p.x + 2 * cos(pa * (PI / 180)), p.y + 2 * sin(pa * (PI / 180))};
        if (!collision_detection(pn)) {
            p = pn;
        }
    }


    for (int i = 0; i < SCREEN_WIDTH; i++) {
        oled_write_pixel(i, UI_HEIGHT, 1);
    }

    // Displays the current game time
    oled_set_cursor(1, 7);
    oled_write_P(PSTR("TIME:"), false);
    oled_write(get_u8_str((timer_elapsed(game_time) - START_TIME_MILLI) / 1000, ' '), false);

    // Displays the players current score
    oled_set_cursor(12, 7);
    oled_write_P(PSTR("SCORE:"), false);
    oled_write(get_u8_str(score, ' '), false);

    raycast(p, pa);
    draw_gun(c.f, shot_timer > 0);
}

// Runs a pseudo-3D raycasting algorithm on the environment around the player
void raycast(vec2 p, int pa) {

    float x3 = p.x;
    float y3 = p.y;
    float x4, y4;

    // Defines the number of rays
    for (int i = 0; i < 128; i+=2) {
        // Calculates the angle at which the ray is projected
        float angle = (i * (fov / 127.0f)) - (fov / 2.0f);

        // Projects the endpoint of the ray
        x4 = p.x + dov * cosf((pa + angle) * (PI / 180));
        y4 = p.y + dov * sinf((pa + angle) * (PI / 180));

        float dist = 100000.0f;
        bool found = false;
        wall cur_wall;
        int cur_edge2pt;

        // Checks if the vector from the camera to the ray's endpoint intersects any walls
        for (int w = 0; w < NUM_WALLS; w++) {
            float x1 = walls[w].points[0].x;
            float y1 = walls[w].points[0].y;
            float x2 = walls[w].points[1].x;
            float y2 = walls[w].points[1].y;

            float denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

            // vectors do not ever intersect
            if (denominator == 0) continue;

            float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
            float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator;

            // Case where the vectors intersect
            if (t > 0 && t < 1 && u > 0) {
                vec2 pt = { x1 + t * (x2 - x1), y1 + t * (y2 - y1) };
                float ptDist2 = dist2(pt, p);

                // Checks if the intersected wall is the closest to the camera
                if (ptDist2 < dist) {
                    dist = ptDist2;
                    found = true;
                    cur_wall = walls[w];
                    cur_edge2pt = dist2(pt, cur_wall.points[0]);
                }
            }
        }

        if (found) {
            int length = 1000 * inv_sqrt(dist);

            // Draws lines at the edges of walls
            int wall_len = 1 / inv_sqrt(dist2(cur_wall.points[0], cur_wall.points[1]));
            cur_edge2pt = 1 / inv_sqrt(cur_edge2pt);
            if (cur_edge2pt < 1 || cur_edge2pt > wall_len - 1) {
                vertical_line(i, length);
                continue;
            }

            if (cur_wall.tex == CHECK) {
                check_line(i, length, cur_edge2pt % 10 < 5);
            }
        //      } else if (cur_wall.tex == STRIPE_H) {
        //
        //      } else if (cur_wall.tex == STRIPE_V) {
        //
        //      } else if (cur_wall.tex == STRIPE_D) {
        //
        //      }

        }
    }    
}

void draw_gun(bool moving, bool show_flash) {

    if (moving || gun_x != GUN_X) {
        gun_x = GUN_X + 8 * sin((double) timer_elapsed(game_time) * 0.005);
    }

    if (moving) {
        gun_y = GUN_Y + 3 + 3 * cos((double) timer_elapsed(game_time) * 0.01);
    
    } else {
        if (gun_x != GUN_X) {
            int inc = abs(GUN_X - gun_x) > 2 ? 2 : 1;
            gun_x += GUN_X > gun_x ? inc : -inc;
        }
        if (gun_y != GUN_Y) gun_y += GUN_Y > gun_y ? 1 : -1;
    } 

    oled_write_bmp_P(gun_bmp_mask, gun_size, GUN_WIDTH, GUN_HEIGHT, gun_x - GUN_WIDTH/2, gun_y - GUN_HEIGHT, true);
    oled_write_bmp_P(gun_bmp, gun_size, GUN_WIDTH, GUN_HEIGHT, gun_x - GUN_WIDTH/2, gun_y - GUN_HEIGHT, false);

    if (show_flash) {
        oled_write_bmp_P(muzzle_flash_bmp, flash_size, FLASH_WIDTH, FLASH_HEIGHT, gun_x - FLASH_WIDTH/2 + 2, gun_y - 3*FLASH_HEIGHT/4 - GUN_HEIGHT, false);
    }
}

// A classic https://en.wikipedia.org/wiki/Fast_inverse_square_root
float inv_sqrt(float num) {
    
    float x2 = num * 0.5f, y = num;
    uint32_t i;
    memcpy(&i, &y, sizeof(float));
    i = 0x5f3759df - ( i >> 1 );
    memcpy(&y, &i, sizeof(float));
    return y * (1.5f - ( x2 * y * y ));
}

void vertical_line(int x, int half_length) {
    
    for (int i = 0; i < half_length; i += 2) {
        oled_write_pixel(x, WALL_OFFSET - i, 1);
        // Ensures that the wall doesnt overlap with the UI
        if (WALL_OFFSET + i < UI_HEIGHT) {
            oled_write_pixel(x, WALL_OFFSET + i, 1);
        }
    }
}

void check_line(int x, int half_length, bool phase) {
   
    int lower = WALL_OFFSET - half_length;
    int upper = WALL_OFFSET + half_length;

    for (int i = lower; i < upper; i+=2) {
        if (i > UI_HEIGHT) break;

        if (phase) {
            if (i == lower || (i >= lower + half_length && i <= lower + 3 * half_length / 2)) {
                i += half_length / 2;
            }

        } else {
            if ((i >= lower + half_length / 2 && i <= lower + half_length) || (i >= lower + 3 * half_length / 2 && i <= upper)) {
                i += half_length / 2;
            }
        }

        oled_write_pixel(x, i, 1);
    }

    oled_write_pixel(x, WALL_OFFSET - half_length, 1);
    if (WALL_OFFSET + half_length < UI_HEIGHT) {
        oled_write_pixel(x, WALL_OFFSET + half_length, 1);
    }
}

bool collision_detection(vec2 p) {

    int collision_dist2 = COLLISION_DIST * COLLISION_DIST;
    for (int i = 0; i < NUM_WALLS; i++) {
        wall w = walls[i];
        vec2 up = sub(p, w.points[0]);
        vec2 uv = sub(w.points[1], w.points[0]);
        vec2 p_proj = add(proj(up, uv), w.points[0]);
        vec2 u_p_proj = sub(p_proj, w.points[0]);

        float k = uv.x != 0 ? u_p_proj.x / uv.x : u_p_proj.y / uv.y;
        float d2 = k <= 0 ? dist2(p, w.points[0]) : k >= 1 ? dist2(p, w.points[1]) : dist2(p, p_proj);
        if (d2 < collision_dist2) return true;
    }

    return false;
}

void oled_write_bmp_P(const char* data, const uint16_t size, int width, int height, int x, int y, bool invert) {

    int row = 0, col = 0;
    for (int i = 0; i < size; i++) {
        uint8_t c = pgm_read_byte(data++);
        for (int j = 0; j < 8; j++) {
            bool px = c & (1 << (7 - j));
            if (px) oled_write_pixel(x + col, y + row, !invert);

            if (col == width - 1) {
                col = 0;
                row++;
                if (row + y >= UI_HEIGHT) return;

                continue;
            }

            col++;
        }
    }
}


// =================== BONGO CAT =================== //

// Animation frame sequences
const char* idle_no_caps[] = {idle_0, idle_0, idle_0, idle_0, idle_1, idle_2, idle_1, idle_0};
const char* slow_no_caps[] = {idle_0, both_down, idle_0};
const char* med_no_caps[]  = {left_down, idle_0, both_down, idle_0, right_down, idle_0};
const char* fast_no_caps[] = {left_down, right_down};

const char* idle_caps[] = {idle_0_open, idle_0_open, idle_0_open, idle_0_open, idle_1_open, idle_2_open, idle_1_open, idle_0_open};
const char* slow_caps[] = {idle_0_open, both_down_open, idle_0_open};
const char* med_caps[]  = {left_down_open, idle_0_open, both_down_open, idle_0_open, right_down_open, idle_0_open};
const char* fast_caps[] = {left_down_open, right_down_open};

// Size of a singular frame in the animation
const size_t frame_size = sizeof(idle_0);

// Framesets for each typing state
const struct frame_set no_caps[] = {
    {.frames = idle_no_caps, .size = sizeof(idle_no_caps) / sizeof(char*), .frame_len = FRAME_LENGTH_SLOW},
    {.frames = slow_no_caps, .size = sizeof(slow_no_caps) / sizeof(char*), .frame_len = FRAME_LENGTH_SLOW},
    {.frames = med_no_caps,  .size = sizeof(med_no_caps) / sizeof(char*),  .frame_len = FRAME_LENGTH_MED},
    {.frames = fast_no_caps, .size = sizeof(fast_no_caps) / sizeof(char*), .frame_len = FRAME_LENGTH_FAST}
};

const struct frame_set caps[] = {
    {.frames = idle_caps, .size = sizeof(idle_caps) / sizeof(char*), .frame_len = FRAME_LENGTH_SLOW},
    {.frames = slow_caps, .size = sizeof(slow_caps) / sizeof(char*), .frame_len = FRAME_LENGTH_SLOW},
    {.frames = med_caps,  .size = sizeof(med_caps) / sizeof(char*),  .frame_len = FRAME_LENGTH_MED},
    {.frames = fast_caps, .size = sizeof(fast_caps) / sizeof(char*), .frame_len = FRAME_LENGTH_FAST}
};

const struct frame_set* frame_sets[] = {no_caps, caps};

uint8_t curr_frame_index = 0;
uint8_t frame_set_index = 0;

static uint8_t curr_wpm = 0;
led_t led_usb_state;
enum oled_state screen_mode = OFF;

controls doom_inputs = {0, 0, 0, 0};

static void render_wpm(void) {

    // Writes the WPM to the screen, and caps if enabled
    oled_set_cursor(0, 7);
    oled_write_P(PSTR("WPM:"), false);
    oled_write(get_u8_str(curr_wpm, ' '), false);
    oled_set_cursor(17, 0);
    if (led_usb_state.caps_lock) {
        oled_write_P(PSTR("CAPS"), false);

    } else {
        oled_write_P(PSTR("    "), false);
    }
}

static void render_bongocat(void) {

    oled_set_cursor(0, 0);
    // Allows for frameset to change to caps state without waiting for end of frame set
    struct frame_set curr_frame_set = frame_sets[led_usb_state.caps_lock][frame_set_index];

    // Updates frame being displayed based on set interval
    if (timer_elapsed(time) > curr_frame_set.frame_len) {

        time = timer_read();

        // Ensures a smoothly animated transition between the different states of the animation
        if (curr_frame_index == curr_frame_set.size - 1) {

            // Assigns frameset based on typing speed
            if (curr_wpm <= IDLE_UPPER_BOUND) {
                frame_set_index = IDLE;

            } else if (curr_wpm <= SLOW_UPPER_BOUND) {
                frame_set_index = SLOW;

            } else if (curr_wpm <= MED_UPPER_BOUND) {
                frame_set_index = MED;

            } else {
                frame_set_index = FAST;
            }

            curr_frame_index = 0;

        } else {
            curr_frame_index++;
        }

        oled_write_raw_P(curr_frame_set.frames[curr_frame_index], frame_size);
    }
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    
    if (screen_mode == DOOM) {
        switch (keycode) {
            case KC_UP:
                doom_inputs.f = record->event.pressed;
                return false;

            case KC_LEFT:
                doom_inputs.l = record->event.pressed;
                return false;

            case KC_RIGHT:
                doom_inputs.r = record->event.pressed;
                return false;

            case KC_SPC:
                doom_inputs.shoot = record->event.pressed;
                return false;
        }

    } else if (!process_record_user(keycode, record)) return false;

    switch (keycode) {
        // Handles the keycode for turning on and off the oled screen
        case KC_OLED_STATE:
            if (record->event.pressed) {
                oled_clear();
                switch (screen_mode) {
                    case OFF:
                        screen_mode = CAT;
                        break;

                    case CAT:
                        screen_mode = DOOM;
                        doom_setup();
                        break;

                    case DOOM:
                        screen_mode = OFF;
                        break;
                }
            }

            return false;
    }
    
    return true;
}

bool oled_task_kb(void) {

    if (!oled_task_user()) return false;

    switch (screen_mode) {
        case CAT:
            curr_wpm = get_current_wpm();
            led_usb_state = host_keyboard_led_state();
            render_wpm();
            render_bongocat();
            break;

        case DOOM:
            if (timer_elapsed(frame_time) > FRAME_TIME_MILLI) {
                frame_time = timer_read();
                doom_update(doom_inputs);
            }
            break;
        
        default:
            break;
    }

    return false;
}