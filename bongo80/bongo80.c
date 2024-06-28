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

// Player information
vec2 p;
int pa;
int shot_timer;

// Game management information
uint32_t time;
uint32_t game_time;
uint32_t frame_time;
uint8_t score;

// Gun animation
int gun_x = GUN_X;
int gun_y = GUN_Y + 10;

// =================== MATH =================== //

float pow2(float x) { return x * x; }

float dot(vec2 u, vec2 v) { return u.x * v.x + u.y * v.y; }

double cross(vec2 u, vec2 v) { return u.x * v.y - u.y * v.x; }

double get_angle(vec2 v1, vec2 v2)
{
	// |A·B| = |A| |B| COS(θ)
	// |A×B| = |A| |B| SIN(θ)
	return atan2(cross(v1, v2), dot(v1, v2));
}

float dist2(vec2 u, vec2 v) { return dot(sub(u, v), sub(u, v)); }

vec2 sub(vec2 u, vec2 v) { return (vec2) {u.x - v.x, u.y - v.y}; }

vec2 add(vec2 u, vec2 v) { return (vec2) {u.x + v.x, u.y + v.y}; }

vec2 proj(vec2 u, vec2 v) { 

    float t = dot(u, v) / dot(v, v);
    return (vec2) {t * v.x, t * v.y};
}

float point_ray_dist2(vec2 p, segment s) {

    vec2 up = sub(p, s.u);
    vec2 uv = sub(s.v, s.u);
    vec2 p_proj = add(proj(up, uv), s.u);
    vec2 u_p_proj = sub(p_proj, s.u);

    float k = uv.x != 0 ? u_p_proj.x / uv.x : u_p_proj.y / uv.y;
    return k <= 0 ? dist2(p, s.u) : k >= 1 ? dist2(p, s.v) : dist2(p, p_proj);
}

// Returns the point of intersection between two line segments
vec2 raycast(segment s1, segment s2, bool* hit) {

    *hit = false;
    vec2 ret = {-1, -1};
    float denominator = (s1.u.x - s1.v.x) * (s2.u.y - s2.v.y) - (s1.u.y - s1.v.y) * (s2.u.x - s2.v.x);

    // vectors do not ever intersect
    if (denominator == 0) return ret;

    float t = ((s1.u.x - s2.u.x) * (s2.u.y - s2.v.y) - (s1.u.y - s2.u.y) * (s2.u.x - s2.v.x)) / denominator;
    float u = -((s1.u.x - s1.v.x) * (s1.u.y - s2.u.y) - (s1.u.y - s1.v.y) * (s1.u.x - s2.u.x)) / denominator;

    // Case where the vectors intersect
    if (t > 0 && t < 1 && u > 0) {
        *hit = true;
        ret = (vec2) { s1.u.x + t * (s1.v.x - s1.u.x), s1.u.y + t * (s1.v.y - s1.u.y) };
    }

    return ret;
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

bool collision_detection(vec2 p) {

    int collision_dist2 = WALL_COLLISION_DIST * WALL_COLLISION_DIST;
    for (int i = 0; i < NUM_WALLS; i++) {
        segment w = walls[i];
        float d2 = point_ray_dist2(p, w);
        if (d2 < collision_dist2) return true;
    }

    collision_dist2 = ENEMY_COLLISION_DIST * ENEMY_COLLISION_DIST;
    for (int i = 0; i < NUM_ENEMIES; i++) {
        enemy e = enemies[i];
        float d2 = dist2(p, e.pos);
        if (d2 < collision_dist2) return true;
    }

    return false;
}


// =================== GRAPHICS =================== //


// 2.5D raycast renderer for the map and entities around the player
void render_map(vec2 p, int pa) {

    segment ray = {p, {0, 0}};

    // Stores the depth at each pixel and the phase of the wall it hit for easier reconstruction
    // depth_buf_info depth_buf[SCREEN_WIDTH / 2];
    // int cursor = 0;

    // Skips every second raycast on walls for performance
    for (int i = 0; i < SCREEN_WIDTH; i += 2) {
        float ray_angle = (i * FOV / SCREEN_WIDTH) - (FOV / 2);
        ray.v.x = p.x + DOV * cosf((pa + ray_angle) * PI / 180);
        ray.v.y = p.y + DOV * sinf((pa + ray_angle) * PI / 180);

        float wall_dist = 100000.0f;
        bool hit_wall = false;
        segment closest_wall;
        depth_buf_info info = {0, 0, 0};
        int wall2pt;

        // Checks if the ray from the camera intersects any walls
        for (int j = 0; j < NUM_WALLS; j++) {
            bool hit = false;
            vec2 pt = raycast(walls[j], ray, &hit);
            if (!hit) continue;

            // Checks if the intersected wall is the closest to the camera
            float ptDist2 = dist2(pt, p);
            if (ptDist2 < wall_dist) {
                wall_dist = ptDist2;
                hit_wall = true;
                closest_wall = walls[j];
                wall2pt = dist2(pt, closest_wall.u);
            }
        }

        if (hit_wall) {
            // Draws lines at the edges of walls
            int wall_len = 1 / inv_sqrt(dist2(closest_wall.u, closest_wall.v));
            wall2pt = 1 / inv_sqrt(wall2pt);

            info.phase = wall2pt % 10 < 5;
            info.length = 1000 * inv_sqrt(wall_dist);
            info.depth = wall_dist;
            if (wall2pt < 2 || wall2pt > wall_len - 2) {
                vertical_line(i, info.length);
            } else {
                check_line(i, info.length, info.phase);
            }
        }

        // depth_buf[cursor++] = info;
    }

    float middle_ray = (SCREEN_WIDTH / 2) * (FOV / 127.0f) - (FOV / 2.0f);

    vec2 ray_vec = {
        p.x + DOV * cosf((pa + middle_ray) * PI / 180),
        p.y + DOV * sinf((pa + middle_ray) * PI / 180)
    };

    // float middle_ray_angle = atan2(ray.v.x - p.x, ray.v.y - p.y);
    for (int i = 0; i < NUM_ENEMIES; i++) {
        enemy e = enemies[i];

        vec2 e_vec = {
            e.pos.x - p.x,
            e.pos.y - p.y
        };

        int determinant = e_vec.x * ray_vec.y - e_vec.y * ray_vec.x;
        float enemy_angle = atan2(determinant, dot(e_vec, ray_vec)) * 180 / PI;
        // oled_set_cursor(0, 0);
        // oled_write(get_u8_str(scale_height, ' '), false);
        // oled_write(get_u8_str(scale_width, ' '), false);
        // oled_write(get_u8_str(y_start, ' '), false);
        // oled_write(get_u16_str(enemy_angle, ' '), false);
        // oled_write(get_u16_str(FOV / 2, ' '), false);
        // oled_write(get_u16_str(FOV / 2, ' '), false);
        if (enemy_angle < FOV / 2 && enemy_angle > -FOV / 2) {
            // Walk across lateral pixels affected by sprite, if any have depth more than enemy distance draw enemy.
            // Redraw walls at those pixels where depth is less than enemy distance.
            float enemy_dist2 = dist2(e.pos, p);
            float enemy_dist = 1 / inv_sqrt(enemy_dist2);
            int scale_height = e.s.height * 50 / enemy_dist;
            int scale_width = e.s.width * 50 / enemy_dist;

            int enemy_screen_x = SCREEN_WIDTH * (enemy_angle + FOV / 2) / FOV;
            int enemy_sreen_l = max(0, enemy_screen_x - scale_width / 2);
            int enemy_sreen_r = min(SCREEN_WIDTH, enemy_screen_x + scale_width / 2);
            bool draw = false;
            for (int j = enemy_screen_l / 2; j < enemy_screen_r / 2; j++) {
                if (depth_buf[j].depth > enemy_dist2) {
                    draw = true;
                    break;
                }
            }

            if (!draw) continue;
            
            int y_start = (/*1000 */ enemy_dist) + WALL_OFFSET;

            // oled_write(get_u16_str(enemy_dist, ' '), false);
            // oled_write(get_u16_str(depth_buf[0].depth, ' '), false);
            oled_write_bmp_P_scaled(e.s, scale_height, scale_width, enemy_screen_l, y_start);

            // for (int k = enemy_l; k < enemy_r; k++) {
            //     depth_buf_info info = depth_buf[k];
            //     if (info.depth < enemy_dist2) {
            //         check_line(k, info.length, info.phase);
            //     }
            // }
            
        // }

        // // Walls are rendered every second lateral pixel, to add detail we render every pixel of entities
        // if (render_enemy(i, ray, enemies[j], hit_wall, wall_dist)) {
        //     ray.v.x = p.x + DOV * cosf((pa + ray_angle) * PI / 180);
        //     ray.v.y = p.y + DOV * sinf((pa + ray_angle) * PI / 180);

        //     render_enemy(i + 1, ray, enemies[j], hit_wall, wall_dist);
        }
    }
}

// bool render_enemy(int screen_x, segment ray, enemy e, bool hit_wall, float wall_dist) {

//     float enemy_dist = dist2(ray.u, e.pos);
//     if (hit_wall && enemy_dist >= wall_dist) return false;

//     // If ray is close to enemy, retrieve slice of enemy sprite based on orthogonal distance to ray 
//     float d = 1 / inv_sqrt(point_ray_dist2(e.pos, ray));
//     if (d > e.width) return false;

//     bool is_left = (ray.u.x - ray.v.x) * (e.pos.y - ray.v.y) - (ray.u.y - ray.v.y) * (e.pos.x - ray.v.x) > 0;
//     int slice = (IMP_WIDTH / 2) + ((is_left ? -1 : 1) * (IMP_WIDTH / 2) * (d / (float) e.width));

//     enemy_dist = inv_sqrt(enemy_dist * 0.4);
//     enemy_dist = enemy_dist > 8 ? 8 : enemy_dist;
//     float slice_height = 1500 * enemy_dist;
//     int y_start = (1000 * enemy_dist) + WALL_OFFSET;

//     oled_write_texture_slice(imp_bmp, imp_bmp_mask, imp_bmp_size, IMP_WIDTH, IMP_HEIGHT, slice, slice_height, screen_x, y_start);
//     return true;
// }

void draw_gun(bool moving, bool show_flash) {

    // Walking animation
    if (moving) {
        gun_x = GUN_X + 8 * sin((double) timer_elapsed(game_time) * 0.005);
        gun_y = GUN_Y + 3 + 3 * cos((double) timer_elapsed(game_time) * 0.01);
    
    // Slowly move gun back to centre when not moving
    } else {
        if (gun_y != GUN_Y) gun_y += GUN_Y > gun_y ? 1 : -1;
        if (gun_x != GUN_X) {
            int inc = abs(GUN_X - gun_x) > 2 ? 2 : 1;
            gun_x += GUN_X > gun_x ? inc : -inc;
        }
    } 

    oled_write_bmp_P(gun_bmp_mask, gun_size, GUN_WIDTH, GUN_HEIGHT, gun_x - GUN_WIDTH/2, gun_y - GUN_HEIGHT, true);
    oled_write_bmp_P(gun_bmp, gun_size, GUN_WIDTH, GUN_HEIGHT, gun_x - GUN_WIDTH/2, gun_y - GUN_HEIGHT, false);

    if (show_flash) {
        oled_write_bmp_P(muzzle_flash_bmp, flash_size, FLASH_WIDTH, FLASH_HEIGHT, gun_x - FLASH_WIDTH/2 + 2, gun_y - 3*FLASH_HEIGHT/4 - GUN_HEIGHT, false);
    }
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


// TODO: Add lr cutoffs? 
// Render walls, save to zbuffer
// Render enemies from left most  ray intersection, add range to zbuffer
// 
void oled_write_bmp_P_scaled(sprite img, int draw_height, int draw_width, int x, int y) {

    if (draw_height < 1 || draw_width < 1) return;

    int row = 0, col = 0;
    for (int i = 0; i < img.size; i++) {
        uint8_t c = pgm_read_byte(img.bmp++);
        uint8_t m = pgm_read_byte(img.mask++);
        for (int j = 0; j < 8; j++) {
            bool px = c & (1 << (7 - j));
            bool pxm = m & (1 << (7 - j));

            int draw_row = draw_height * row / img.height;
            int draw_row_lim = draw_height * (row + 1) / img.height;
            int draw_col = draw_width * col / img.width;
            int draw_col_lim = draw_width * (col + 1) / img.width;

            for (int k = draw_row; k < draw_row_lim; k++) {
                if (y + k >= UI_HEIGHT) return;

                for (int l = draw_col; l < draw_col_lim; l++) {
                    if (px) oled_write_pixel(x + l, y + k, true);
                    if (pxm) oled_write_pixel(x + l, y + k, false);
                }
            }

            if (++col == img.width) {
                row++;
                col = 0;
            }
        }
    }
}

void oled_write_texture_slice(const char* data, const char* mask, const uint16_t size, int text_width, int text_height, int slice_col, int slice_height, int x, int y_start) {
    
    int height_written = 0;
    int byte_offset = slice_col / 8;
    int bit_offset = slice_col - byte_offset * 8;  

    for (int row = 0; row < text_height; row++) {
        data += byte_offset;
        mask += byte_offset;

        uint8_t c = pgm_read_byte(data);
        uint8_t m = pgm_read_byte(mask);

        bool px = c & (1 << (7 - bit_offset));
        bool pxm = m & (1 << (7 - bit_offset));
        
        while (height_written < slice_height * row / text_height) {
            if (y_start - slice_height + height_written >= UI_HEIGHT) return;
            if (px) oled_write_pixel(x, y_start + height_written - slice_height, true);
            if (pxm) oled_write_pixel(x, y_start + height_written - slice_height, false);
            height_written++;
        }

        byte_offset = (text_width + bit_offset) / 8;
        bit_offset = text_width + bit_offset - byte_offset * 8;
    }
}


// =================== GAME LOGIC =================== //


void doom_setup(void) {

    // Runs intro sequence
    oled_write_bmp_P(doom_logo, doom_logo_size, LOGO_WIDTH, LOGO_HEIGHT, 0, 0, false);
    game_time = timer_read();

    // Initializes player state
    p = (vec2) {20.0f, 20.0f};
    pa = 0;
    shot_timer = 0;
    score = 0;
}

void doom_update(controls c) {

    if (timer_elapsed(game_time) < START_TIME_MILLI) return;

    oled_clear();
    if (c.shoot && shot_timer == 0) shot_timer = 5;
    if (shot_timer > 0) shot_timer--;

    if (c.l) {
        pa -= pa - ROTATION_SPEED < 0 ? ROTATION_SPEED + 360 : ROTATION_SPEED;
    }

    if (c.r) {
        pa += pa + ROTATION_SPEED >= 360 ? ROTATION_SPEED - 360 : ROTATION_SPEED;
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
    // oled_set_cursor(1, 7);
    // oled_write_P(PSTR("TIME:"), false);
    // oled_write(get_u16_str(timer_elapsed(game_time), ' '), false);

    // // Displays the players current score
    // oled_set_cursor(12, 7);
    // oled_write_P(PSTR("SCORE:"), false);
    // oled_write(get_u8_str(score, ' '), false);

    render_map(p, pa);
    draw_gun(c.f, shot_timer > 0);
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