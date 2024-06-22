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
    if (c.shoot && shot_timer == 0) shot_timer = 5;
    if (shot_timer > 0) shot_timer--;

    if (c.l) {
        pa = (pa - rotSpeed < 0 ? pa - rotSpeed + 360 : pa - rotSpeed);
    }

    if (c.r) {
        pa = (pa + rotSpeed >= 360 ? pa + rotSpeed - 360 : pa + rotSpeed);
    }

    if (c.f) {
        vec2 pnx = {p.x + 2 * cos(pa * (PI / 180)), p.y};
        vec2 pny = {p.x, p.y + 2 * sin(pa * (PI / 180))};
        if (!collision_detection(pnx)) p.x = pnx.x;
        if (!collision_detection(pny)) p.y = pny.y;
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

    render_map(p, pa);
    draw_gun(c.f, shot_timer > 0);
}

// Runs a pseudo-3D raycasting algorithm on the environment around the player
void render_map(vec2 p, int pa) {

    // Defines the number of rays
    for (int i = 0; i < 128; i+=2) {
        // Calculates the angle at which the ray is projected
        float angle = (i * (fov / 127.0f)) - (fov / 2.0f);

        // Projects the endpoint of the ray
        vec2 endpoint = {
            p.x + dov * cosf((pa + angle) * (PI / 180)),
            p.y + dov * sinf((pa + angle) * (PI / 180))
        };

        float dist = 100000.0f;
        bool found = false;
        wall cur_wall;
        int cur_edge2pt;

        // Checks if the vector from the camera to the ray's endpoint intersects any walls
        for (int w = 0; w < NUM_WALLS; w++) {
            bool hit = false;
            vec2 pt = raycast(walls[w].points[0], walls[w].points[1], p, endpoint, &hit);
            if (hit) {
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
            if (cur_edge2pt < 2 || cur_edge2pt > wall_len - 2) {
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

        for (int j = 0; j < NUM_ENEMIES; j++) {
            vec2 e = enemies[j].pos;
            float enemy_dist = dist2(p, e);
            if (found && enemy_dist >= dist) continue;

            float d = 1 / inv_sqrt(point_ray_dist2(e, p, endpoint));
            if (d > 8) continue;

            bool is_left = (p.x - endpoint.x)*(e.y - endpoint.y) - (p.y - endpoint.y)*(e.x - endpoint.x) > 0;
            int slice = (IMP_WIDTH / 2) + ((is_left ? -1 : 1) * (IMP_WIDTH / 2) * (d / 8));

            enemy_dist = inv_sqrt(enemy_dist);
            float slice_height = 2000 * enemy_dist;
            int y_start = (700 * enemy_dist) + WALL_OFFSET;
            if (y_start > 70) y_start = 70;
            oled_set_cursor(0, 0);
            oled_write(get_u16_str(slice_height, ' '), false);
            oled_write(get_u16_str(y_start, ' '), false);

            oled_write_texture_slice(imp_bmp, imp_bmp_mask, imp_bmp_size, IMP_WIDTH / 8, IMP_HEIGHT, slice, slice_height, i, y_start);
            // oled_write_texture_slice(imp_bmp, imp_bmp_mask, imp_bmp_size, IMP_WIDTH / 8, IMP_HEIGHT, slice, i+1, 2);
        }
    }    
}

vec2 raycast(vec2 ls1, vec2 ls2, vec2 r1, vec2 r2, bool* hit) {

    *hit = false;
    vec2 ret = {-1, -1};
    float denominator = (ls1.x - ls2.x) * (r1.y - r2.y) - (ls1.y - ls2.y) * (r1.x - r2.x);

    // vectors do not ever intersect
    if (denominator == 0) return ret;

    float t = ((ls1.x - r1.x) * (r1.y - r2.y) - (ls1.y - r1.y) * (r1.x - r2.x)) / denominator;
    float u = -((ls1.x - ls2.x) * (ls1.y - r1.y) - (ls1.y - ls2.y) * (ls1.x - r1.x)) / denominator;

    // Case where the vectors intersect
    if (t > 0 && t < 1 && u > 0) {
        *hit = true;
        ret = (vec2) { ls1.x + t * (ls2.x - ls1.x), ls1.y + t * (ls2.y - ls1.y) };
    }

    return ret;
}

void draw_gun(bool moving, bool show_flash) {

    if (moving) {
        gun_x = GUN_X + 8 * sin((double) timer_elapsed(game_time) * 0.005);
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

float point_ray_dist2(vec2 p, vec2 u, vec2 v) {

    vec2 up = sub(p, u);
    vec2 uv = sub(v, u);
    vec2 p_proj = add(proj(up, uv), u);
    vec2 u_p_proj = sub(p_proj, u);

    float k = uv.x != 0 ? u_p_proj.x / uv.x : u_p_proj.y / uv.y;
    return k <= 0 ? dist2(p, u) : k >= 1 ? dist2(p, v) : dist2(p, p_proj);
}

bool collision_detection(vec2 p) {

    int collision_dist2 = COLLISION_DIST * COLLISION_DIST;
    for (int i = 0; i < NUM_WALLS; i++) {
        wall w = walls[i];
        float d2 = point_ray_dist2(p, w.points[0], w.points[1]);
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

void oled_write_texture_slice(const char* data, const char* mask, const uint16_t size, int row_bytes, int text_height, int slice_col, int height, int x, int y) {
    
    height = height > 50 ? 50 : height;
    int height_written = 0;
    for (int row = 0; row < text_height; row++) {
        int byte_offset = slice_col / 8;
        data += byte_offset;
        mask += byte_offset;
        int rem = slice_col - byte_offset * 8;

        uint8_t c = pgm_read_byte(data);
        uint8_t m = pgm_read_byte(mask);
        bool px = c & (1 << (7 - rem));
        bool pxm = m & (1 << (7 - rem));

        while (height_written < height * row / text_height) {
            if (px) oled_write_pixel(x, y - height + height_written, true);
            if (pxm) oled_write_pixel(x, y - height + height_written, false);
            height_written++;
        }

        data += row_bytes - byte_offset;
        mask += row_bytes - byte_offset;
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