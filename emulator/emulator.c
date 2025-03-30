#include <SDL2/SDL.h>
#include "../bongo80/doom.h"

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
                              SCR_WIDTH, SCR_HEIGHT, 0);

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
int timer_read();
int timer_elapsed32();
int oled_set_cursor(int, int);
int oled_write(long, int);
int oled_clear();
int get_current_wpm();
int host_keyboard_led_state();


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

        int dx = 0, dz = 0;
        const uint8_t* currentKeyStates = SDL_GetKeyboardState(NULL);
        if (currentKeyStates[SDL_SCANCODE_UP]) dz++;
        if (currentKeyStates[SDL_SCANCODE_DOWN]) dz--;
        if (currentKeyStates[SDL_SCANCODE_LEFT]) dx--;
        if (currentKeyStates[SDL_SCANCODE_RIGHT]) dx++;

        Vec3 dd = {0, 0, 0}, headingXNorm = {0, 0, 0}, headingZNorm = {0, 0, 0};
        if (dx != 0)
        {
            headingXNorm = (Vec3) {0, heading.y, 0}.cross({0, 0, heading.z});
            headingXNorm.normalize();
            if (headingXNorm.x == 0 && headingXNorm.y == 0 && headingXNorm.z == 0)
            {
                headingXNorm.x += dx;
            }
            else
            {
                headingXNorm = headingXNorm * dx;
            }
        }

        if (dz != 0)
        {
            headingZNorm = (Vec3) {heading.x, 0, 0}.cross({0, heading.y, 0});
            headingZNorm.normalize();
            if (headingZNorm.x == 0 && headingZNorm.y == 0 && headingZNorm.z == 0)
            {
                headingZNorm.z += dz;
            }
            else
            {
                headingZNorm = headingZNorm * dz;
            }
        }

        dd = headingZNorm + headingXNorm;
        dd.normalize();
        position.add(dd * STEP_SIZE);

        getFrame(fb, slr, position, heading, worldMesh);
        drawFrame(renderer, fb);
        // SDL_Delay(10);
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