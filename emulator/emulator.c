#ifndef USE_EMULATOR
    #define USE_EMULATOR
#endif

#include <SDL2/SDL.h>
#include <time.h>
#include "emulator.h"
#include "../bongo80/doom.h"
#include "../bongo80/bongo80.h"

static int screen_mode = DOOM;
controls doom_inputs = {0, 0, 0, 0, 0};

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
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC_RAW , & t); // change CLOCK_MONOTONIC_RAW to CLOCK_MONOTONIC on non linux computers
    return t.tv_sec * 1000 + (t.tv_nsec + 500000) / 1000000;
}

uint32_t timer_elapsed32() {
    return timer_read();
}

int oled_set_cursor(int, int);
int oled_write(const char* str, int smth) {
    return 1;
}

int oled_write_P(const char* str, int smth) {
    return oled_write(str, smth);
}

int oled_clear();
int get_current_wpm();
int host_keyboard_led_state();

char* get_u8_str(uint8_t val, char pad) {
    return "test";
};

char* get_u16_str(uint16_t val, char pad) {
    return "test";
};

uint8_t pgm_read_byte(const void* addr)
{
    return *(uint8_t*) addr;
}

uint32_t frame_time = 0;

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

    screen_mode = DOOM;
    doom_setup();

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
    return 0;
}