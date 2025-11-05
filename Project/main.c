#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "leds.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "motors.h"
#include "sensors/battery_level.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "selector.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// ============================================================================
//                                TASK 1 (AVOIDANCE)
// ============================================================================
typedef enum { DRIVE = 0, PIVOT_LEFT, PIVOT_RIGHT } AvoidState;

const int T1_THR_FRONT   = 700;
const int T1_REL_FRONT   = 500;
const int T1_THR_SIDE    = 600;
const int T1_DRIVE_SPEED = 550;
const int T1_PIVOT_SPEED = 650;
const int T1_SIDE_GAIN   = 1;
const int T1_HOLD_CYCLES = 3;
const int LOOP_DT_MS     = 80;

static void set_rgb_task1(AvoidState s)
{
    clear_leds();
    switch(s){
        case DRIVE:       set_rgb_led(0,0,1,0); break; // Green
        case PIVOT_LEFT:  set_rgb_led(0,0,0,1); break; // Blue
        case PIVOT_RIGHT: set_rgb_led(0,1,0,0); break; // Red
        default: break;
    }
}
static void update_ring_leds_task1(int F,int L,int R)
{
    set_led(LED1, (F > T1_THR_FRONT) ? 1 : 0); // Front
    set_led(LED3, (R > T1_THR_SIDE)  ? 1 : 0); // Right
    set_led(LED7, (L > T1_THR_SIDE)  ? 1 : 0); // Left
    set_led(LED5, 0);                          // Back unused
}
void run_obstacle_avoidance(void)
{
    AvoidState state = DRIVE;
    int hold_counter = 0;
    set_rgb_task1(state);

    while (get_selector() == 0) {
        int p0=get_prox(0),p1=get_prox(1),p2=get_prox(2);
        int p5=get_prox(5),p6=get_prox(6),p7=get_prox(7);
        int F=(p0+p7)/2, L=(p6+p5+p7)/3, R=(p0+p1+p2)/3;

        update_ring_leds_task1(F,L,R);

        switch(state){
        case DRIVE:
            if(F>=T1_THR_FRONT){
                state=(L>=R)?PIVOT_RIGHT:PIVOT_LEFT;
                hold_counter=T1_HOLD_CYCLES;
                set_rgb_task1(state);
                left_motor_set_speed((state==PIVOT_LEFT)?-T1_PIVOT_SPEED: T1_PIVOT_SPEED);
                right_motor_set_speed((state==PIVOT_LEFT)? T1_PIVOT_SPEED:-T1_PIVOT_SPEED);
            }else{
                int ls=T1_DRIVE_SPEED,rs=T1_DRIVE_SPEED;
                if(L>T1_THR_SIDE && L>R+50){ ls+=T1_SIDE_GAIN*(L-T1_THR_SIDE); rs-=T1_SIDE_GAIN*(L-T1_THR_SIDE); }
                else if(R>T1_THR_SIDE && R>L+50){ ls-=T1_SIDE_GAIN*(R-T1_THR_SIDE); rs+=T1_SIDE_GAIN*(R-T1_THR_SIDE); }
                if(ls>1000)ls=1000;if(rs>1000)rs=1000;if(ls<0)ls=0;if(rs<0)rs=0;
                left_motor_set_speed(ls); right_motor_set_speed(rs);
            }
            break;
        case PIVOT_LEFT:
        case PIVOT_RIGHT:
            left_motor_set_speed((state==PIVOT_LEFT)?-T1_PIVOT_SPEED: T1_PIVOT_SPEED);
            right_motor_set_speed((state==PIVOT_LEFT)? T1_PIVOT_SPEED:-T1_PIVOT_SPEED);
            if(F<T1_REL_FRONT){
                if(hold_counter>0) hold_counter--;
                else{ state=DRIVE; set_rgb_task1(state); }
            }else hold_counter=T1_HOLD_CYCLES;
            break;
        }
        chThdSleepMilliseconds(LOOP_DT_MS);
    }
    clear_leds();
    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

// ============================================================================
//                                TASK 2 (CHASE)
// ============================================================================
typedef enum { SEARCH=0, TURN, APPROACH, STOP, BACKOFF } ChaseState;

const uint16_t SEARCH_RANGE_THR = 75;
const uint16_t DESIRED_MIN_MM   = 45;
const uint16_t DESIRED_MAX_MM   = 65;
const uint16_t BACKOFF_DIST_MM  = 40;
const uint16_t SAFE_DIST_MM     = 50;

const int DETECT_THR     = 300;
const int CENTER_THR     = 150;
const int LOST_THR       = 200;
const int TRACK_DIFF_THR = 120;

const int SEARCH_SPEED   = 250;
const int TURN_SPEED     = 450;
const int APPROACH_FAST  = 700;
const int APPROACH_SLOW  = 250;
const int BACKOFF_SPEED  = -300;
const int TRACK_SPEED    = 250;

static inline int front_read(int p0,int p7,int p1,int p6){return (45*p7+45*p0+5*p6+5*p1)/100;}
static inline int left_read (int p6,int p5,int p7){return (60*p6+30*p5+10*p7)/100;}
static inline int right_read(int p1,int p2,int p0){return (60*p1+30*p2+10*p0)/100;}
static inline int back_read (int p3,int p4){return (50*p3+50*p4)/100;}

static void set_led_task2(ChaseState s)
{
    clear_leds();
    switch(s){
        case SEARCH:  set_rgb_led(0,0,0,1); break; // Blue
        case TURN:    set_rgb_led(0,0,1,1); break; // Cyan
        case APPROACH:set_rgb_led(0,0,1,0); break; // Green
        case STOP:    set_rgb_led(0,1,1,0); break; // Yellow
        case BACKOFF: set_rgb_led(0,1,0,0); break; // Red
        default: break;
    }
}

void run_object_chasing(void)
{
    ChaseState state = SEARCH;
    set_led_task2(state);

    while (get_selector() == 1) {
        uint16_t dist_mm = VL53L0X_get_dist_mm();
        if(dist_mm==0) dist_mm=9999;

        int p[8]; for(int i=0;i<8;i++) p[i]=get_prox(i);
        int F=front_read(p[0],p[7],p[1],p[6]);
        int L=left_read(p[6],p[5],p[7]);
        int R=right_read(p[1],p[2],p[0]);
        int B=back_read(p[3],p[4]);
        int max_idx=0; for(int i=1;i<8;i++) if(p[i]>p[max_idx]) max_idx=i;
        int max_val=p[max_idx];

        switch(state){
        case SEARCH:
            left_motor_set_speed( SEARCH_SPEED);
            right_motor_set_speed(-SEARCH_SPEED);
            if(dist_mm<SEARCH_RANGE_THR || max_val>DETECT_THR){state=TURN; set_led_task2(state);}
            break;
        case TURN:
            if(dist_mm>=SEARCH_RANGE_THR && max_val<DETECT_THR){state=SEARCH; set_led_task2(state); break;}
            if(max_idx==3||max_idx==4){left_motor_set_speed( TURN_SPEED); right_motor_set_speed(-TURN_SPEED);}
            else if(max_idx==5||max_idx==6||max_idx==7){left_motor_set_speed(-TURN_SPEED); right_motor_set_speed( TURN_SPEED);}
            else if(max_idx==0||max_idx==1||max_idx==2){left_motor_set_speed( TURN_SPEED); right_motor_set_speed(-TURN_SPEED);}
            if(max_idx==0||max_idx==7){left_motor_set_speed(0); right_motor_set_speed(0); state=APPROACH; set_led_task2(state);}
            break;
        case APPROACH:
            if(dist_mm<=DESIRED_MIN_MM){left_motor_set_speed(0); right_motor_set_speed(0); state=STOP; set_led_task2(state); break;}
            if(dist_mm>DESIRED_MAX_MM && dist_mm<1200){
                int sp=APPROACH_SLOW+(dist_mm-DESIRED_MAX_MM)*(APPROACH_FAST-APPROACH_SLOW)/(SEARCH_RANGE_THR-DESIRED_MAX_MM);
                if(sp>APPROACH_FAST)sp=APPROACH_FAST; if(sp<APPROACH_SLOW)sp=APPROACH_SLOW;
                left_motor_set_speed(sp); right_motor_set_speed(sp);
            }else{ left_motor_set_speed(0); right_motor_set_speed(0); state=STOP; set_led_task2(state);}
            if(dist_mm>=1200 && max_val<LOST_THR){state=SEARCH; set_led_task2(state);}
            break;
        case STOP:
            left_motor_set_speed(0); right_motor_set_speed(0);
            if(max_val>DETECT_THR){
                if(max_idx==5||max_idx==6||max_idx==7){left_motor_set_speed(-TRACK_SPEED); right_motor_set_speed( TRACK_SPEED);}
                else if(max_idx==0||max_idx==1||max_idx==2){left_motor_set_speed( TRACK_SPEED); right_motor_set_speed(-TRACK_SPEED);}
                else if(max_idx==3||max_idx==4){left_motor_set_speed( TURN_SPEED); right_motor_set_speed(-TURN_SPEED);}
                else{left_motor_set_speed(0); right_motor_set_speed(0);}
            }
            if(dist_mm>DESIRED_MAX_MM && dist_mm<1200){state=APPROACH; set_led_task2(state);}
            if(dist_mm<BACKOFF_DIST_MM){state=BACKOFF; set_led_task2(state);}
            if(dist_mm>=1200 && max_val<LOST_THR){state=SEARCH; set_led_task2(state);}
            break;
        case BACKOFF:
            left_motor_set_speed(BACKOFF_SPEED); right_motor_set_speed(BACKOFF_SPEED);
            if(dist_mm>SAFE_DIST_MM){left_motor_set_speed(0); right_motor_set_speed(0); state=STOP; set_led_task2(state);}
            break;
        }
        chThdSleepMilliseconds(LOOP_DT_MS);
    }
    clear_leds();
    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

// ============================================================================
//                                 MAIN
// ============================================================================
int main(void)
{
    halInit(); chSysInit(); mpu_init();
    messagebus_init(&bus,&bus_lock,&bus_condvar);

    clear_leds();
    spi_comm_start();
    proximity_start(0); calibrate_ir();
    VL53L0X_start();
    motors_init();
    battery_level_start(); get_battery_percentage();
    dac_start(); playMelodyStart();

    while (1) {
        int sel = get_selector();
        if(sel==0) run_obstacle_avoidance();
        else if(sel==1) run_object_chasing();
        else {
            clear_leds();
            left_motor_set_speed(0);
            right_motor_set_speed(0);
        }
        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard=STACK_CHK_GUARD;
void __stack_chk_fail(void){ chSysHalt("Stack smashing detected"); }
