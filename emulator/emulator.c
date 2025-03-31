#define USE_EMULATOR

#include <SDL2/SDL.h>
#include "../bongo80/doom.h"
#include "../bongo80/bongo80.h"

int initSDL(SDL_Window** window, SDL_Renderer** renderer)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0) 
    {
        printf("Failed to initialise\n");
        return 0;
    }

    *window = SDL_CreateWindow("QMK Emulator", 
                              SDL_WINDOWPOS_CENTERED,
                              SDL_WINDOWPOS_CENTERED,
                              128, 64, 0);

    if (!*window)
    {
        printf("Failed to create window\n");
        return 0;
    }

    *renderer = SDL_CreateRenderer(*window, 0, 0);
    if (!*renderer)
    {
        printf("Failed to create renderer\n");
        return 0;
    }

    return 1;
}

int oled_write_pixel(int, int, int);
int timer_read() {
    return millis();
}

int timer_elapsed32();
int oled_set_cursor(int, int);
int oled_write(long, int);
int oled_clear();
int get_current_wpm();
int host_keyboard_led_state();

uint8_t pgm_read_byte(const void* addr)
{
    return *(uint8_t*) addr;
}

int main()
{
    SDL_Window* window;
    SDL_Event event;
    SDL_Renderer* renderer;

    if (!initSDL(&window, &renderer)) 
    {
        printf("Failed to initialise SDL!\n");
        goto clean;
    }

    uint32_t frame_time = timer_read();
    enum oled_state screen_mode = DOOM;
    controls doom_inputs = {0, 0, 0, 0, 0};
    while (1)
    {
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
                case SDL_QUIT:
                goto clean;
            }
        }
        
        controls doom_inputs;
        const uint8_t* currentKeyStates = SDL_GetKeyboardState(NULL);
        if (currentKeyStates[SDL_SCANCODE_UP]) 
            doom_inputs.u = true;

        if (currentKeyStates[SDL_SCANCODE_DOWN])
            doom_inputs.d = true;

        if (currentKeyStates[SDL_SCANCODE_LEFT])
            doom_inputs.l = true;

        if (currentKeyStates[SDL_SCANCODE_RIGHT])
            doom_inputs.r = true;
        
        if (currentKeyStates[SDLK_SPACE])
            doom_inputs.shoot = true;


        switch (screen_mode) {
            case CAT:
                render_wpm();
                render_bongocat();
                break;
    
            case DOOM:
                if (timer_elapsed32(frame_time) > FRAME_TIME_MILLI) {
                    doom_update(doom_inputs);
                    frame_time = timer_read();
                }
                break;
            
            default:
                break;
        }
    }   

clean:
    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(renderer);
    SDL_Quit();
    if (fb && fb->buf)
    {
        for (int i = 0; i < fb->width; i++)
        {
            free(fb->buf[i]);
        }

        free(fb->buf);
        free(fb);
    }

    if (slr && slr->rays)
    {
        for (int i = 0; i < slr->width; i++)
        {
            free(slr->rays[i]);
        }

        free(slr->rays);
        free(slr);
    }

    return 0;
}


bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    
    if (screen_mode == DOOM) {
        switch (keycode) {
            case KC_UP:
                doom_inputs.u = record->event.pressed;
                return false;
            
            case KC_DOWN:
                doom_inputs.d = record->event.pressed;
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
                        doom_dispose();
                        screen_mode = OFF;
                        break;
                }
            }

            return false;
    }
    
    return true;
}

bool oled_task_kb(void) {

    switch (screen_mode) {
        case CAT:
            curr_wpm = get_current_wpm();
            led_usb_state = host_keyboard_led_state();
            render_wpm();
            render_bongocat();
            break;

        case DOOM:
            if (timer_elapsed32(frame_time) > FRAME_TIME_MILLI) {
                doom_update(doom_inputs);
                frame_time = timer_read();
            }
            break;
        
        default:
            break;
    }

    return false;
}