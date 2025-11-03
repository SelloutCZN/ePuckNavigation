#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

// LED headers
#include "leds.h"
#include "spi_comm.h"

// Proximity sensor headers
#include "sensors/proximity.h"

// Distance sensor header
#include "sensors/VL53L0X/VL53L0X.h"

// Motor header
#include "motors.h"

// Battery sensor header
#include "sensors/battery_level.h"

// Speakers & Audio headers
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// -------------------- STATE MACHINE DEFINITIONS --------------------
typedef enum { DRIVE = 0, PIVOT_LEFT, PIVOT_RIGHT } AvoidState;

// -------------------- CONFIGURABLE PARAMETERS --------------------
const int THR_FRONT     = 700;   // start turning if front >= this
const int REL_FRONT     = 500;   // stop turning when front < this
const int THR_SIDE      = 600;   // sensitivity for side avoidance
const int DRIVE_SPEED   = 550;   // base forward speed
const int PIVOT_SPEED   = 650;   // in-place turning speed
const int SIDE_TURN_GAIN = 1;    // multiply by side difference for gentle steer
const int HOLD_CYCLES   = 3;     // hold pivot for extra cycles after clear
const int LOOP_DT_MS    = 80;    // control loop period

// -------------------- HELPER FUNCTIONS --------------------
static inline int front_read(int p0, int p7, int p1, int p6) {
    // weight frontal sensors most
    return (45*p7 + 45*p0 + 5*p6 + 5*p1) / 100;
}

static inline int left_read(int p6, int p5, int p7) {
    return (60*p6 + 30*p5 + 10*p7) / 100;
}

static inline int right_read(int p1, int p2, int p0) {
    return (60*p1 + 30*p2 + 10*p0) / 100;
}

// -------------------- MAIN PROGRAM --------------------
int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Initialize LEDs and SPI comm
    clear_leds();
    spi_comm_start();

    // Start proximity sensors and calibrate
    proximity_start(0);
    calibrate_ir();

    // Start distance sensor (optional)
    VL53L0X_start();

    // Initialize motors
    motors_init();
    left_motor_set_speed(0);
    right_motor_set_speed(0);

    // Start battery monitor
    battery_level_start();
    get_battery_percentage();

    // Start DAC + melodies (optional)
    dac_start();
    playMelodyStart();

    // Initialize state variables
    AvoidState state = DRIVE;
    int hold_counter = 0;

    // Main control loop
    while (1) {
        // Read all 8 IR proximity sensors
        int p0 = get_prox(0), p1 = get_prox(1), p2 = get_prox(2), p3 = get_prox(3);
        int p4 = get_prox(4), p5 = get_prox(5), p6 = get_prox(6), p7 = get_prox(7);

        // Calculate sector readings
        int F = front_read(p0, p7, p1, p6);
        int L = left_read(p6, p5, p7);
        int R = right_read(p1, p2, p0);

        switch (state) {
        case DRIVE:
            if (F >= THR_FRONT) {
                // Obstacle detected ahead — pivot in place
                state = (L >= R) ? PIVOT_RIGHT : PIVOT_LEFT;
                hold_counter = HOLD_CYCLES;

                left_motor_set_speed( (state == PIVOT_LEFT) ? -PIVOT_SPEED :  PIVOT_SPEED );
                right_motor_set_speed((state == PIVOT_LEFT) ?  PIVOT_SPEED : -PIVOT_SPEED );
            } else {
                // ---- Forward driving with gentle side avoidance ----
                int left_speed  = DRIVE_SPEED;
                int right_speed = DRIVE_SPEED;

                // If one side is too close to a wall, bias away
                if (L > THR_SIDE && L > R + 50) {
                    // too close on left → steer slightly right
                    left_speed  += SIDE_TURN_GAIN * (L - THR_SIDE);
                    right_speed -= SIDE_TURN_GAIN * (L - THR_SIDE);
                } else if (R > THR_SIDE && R > L + 50) {
                    // too close on right → steer slightly left
                    left_speed  -= SIDE_TURN_GAIN * (R - THR_SIDE);
                    right_speed += SIDE_TURN_GAIN * (R - THR_SIDE);
                }

                // clamp speeds to safe limits
                if (left_speed  > 1000) left_speed  = 1000;
                if (right_speed > 1000) right_speed = 1000;
                if (left_speed  < 0)    left_speed  = 0;
                if (right_speed < 0)    right_speed = 0;

                left_motor_set_speed(left_speed);
                right_motor_set_speed(right_speed);
            }
            break;

        case PIVOT_LEFT:
        case PIVOT_RIGHT:
            // Keep pivoting in place until front clear
            left_motor_set_speed( (state == PIVOT_LEFT) ? -PIVOT_SPEED :  PIVOT_SPEED );
            right_motor_set_speed((state == PIVOT_LEFT) ?  PIVOT_SPEED : -PIVOT_SPEED );

            if (F < REL_FRONT) {
                if (hold_counter > 0) {
                    hold_counter--;
                } else {
                    state = DRIVE;
                }
            } else {
                hold_counter = HOLD_CYCLES;
            }
            break;
        }

        chThdSleepMilliseconds(LOOP_DT_MS);
    }
}

// -------------------- STACK GUARD --------------------
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
