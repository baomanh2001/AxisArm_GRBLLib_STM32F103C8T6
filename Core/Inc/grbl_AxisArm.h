/*
 * grbl_AxisArm.h
 *
 *  Created on: May 5, 2026
 *      Author: DELL
 */

#ifndef INC_GRBL_AXISARM_H_
#define INC_GRBL_AXISARM_H_

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*============================================================================
 *                    BASIC CONFIGURATION
 *============================================================================*/

#define CNC_NUM_AXES                4       /* X, Y, Z, V */
#define CNC_USE_ENABLE_PIN          0
#define CNC_TIMER_SELECT            2
#define CNC_UART_SELECT             1
#define CNC_UART_BAUDRATE           115200
#define CNC_SYSTEM_CLOCK            72000000UL

#define CNC_STEP_PULSE_US           3
#define CNC_BASE_TICK_US            20

/*============================================================================
 *                    STEP PULSE CONFIGURATION
 *============================================================================*/

/** Step pulse duty cycle (0-100%)
 *  90 = HIGH 90% of period, LOW 10%
 */
#define CNC_STEP_DUTY_PERCENT       90
#define CNC_STEP_MIN_LOW_US         2
#define CNC_STEP_MIN_HIGH_US        2

/*============================================================================
 *                    FEATURE ENABLES
 *============================================================================*/

#define CNC_USE_DMA_UART            1
#define CNC_USE_COMMAND_QUEUE       1
#define CNC_USE_PLANNER             1
#define CNC_USE_BACKLASH            0
#define CNC_USE_ACCELERATION        1
#define CNC_USE_HOMING              1
#define CNC_USE_LIMIT_SWITCH        1
#define CNC_USE_RELAY               1
#define CNC_AUTO_REPORT_ENABLE      1

/** Enable GUI Protocol (GOTO/SET/RETURN/Presets) */
#define CNC_USE_GUI_PROTOCOL        1

/*============================================================================
 *                    DMA UART CONFIGURATION
 *============================================================================*/

#if CNC_USE_DMA_UART

#define UART_DMA_BUFFER_SIZE        256

#if (CNC_UART_SELECT == 1)
    #define CNC_UART_DMA            DMA1
    #define CNC_UART_DMA_CHANNEL    DMA1_Channel5
    #define CNC_UART_DMA_IRQn       DMA1_Channel5_IRQn
    #define CNC_UART_DMA_IRQHandler DMA1_Channel5_IRQHandler
    #define CNC_UART_DMA_FLAG_TC    DMA_ISR_TCIF5
    #define CNC_UART_DMA_FLAG_HT    DMA_ISR_HTIF5
#elif (CNC_UART_SELECT == 2)
    #define CNC_UART_DMA            DMA1
    #define CNC_UART_DMA_CHANNEL    DMA1_Channel6
    #define CNC_UART_DMA_IRQn       DMA1_Channel6_IRQn
    #define CNC_UART_DMA_IRQHandler DMA1_Channel6_IRQHandler
    #define CNC_UART_DMA_FLAG_TC    DMA_ISR_TCIF6
    #define CNC_UART_DMA_FLAG_HT    DMA_ISR_HTIF6
#elif (CNC_UART_SELECT == 3)
    #define CNC_UART_DMA            DMA1
    #define CNC_UART_DMA_CHANNEL    DMA1_Channel3
    #define CNC_UART_DMA_IRQn       DMA1_Channel3_IRQn
    #define CNC_UART_DMA_IRQHandler DMA1_Channel3_IRQHandler
    #define CNC_UART_DMA_FLAG_TC    DMA_ISR_TCIF3
    #define CNC_UART_DMA_FLAG_HT    DMA_ISR_HTIF3
#endif

#endif /* CNC_USE_DMA_UART */

/*============================================================================
 *                    COMMAND QUEUE CONFIGURATION
 *============================================================================*/

#if CNC_USE_COMMAND_QUEUE
    #define CMD_QUEUE_SIZE          16
    #define CMD_MAX_LENGTH          80
#endif

/*============================================================================
 *                    MOTION PLANNER CONFIGURATION
 *============================================================================*/

#if CNC_USE_PLANNER
    #define PLANNER_BUFFER_SIZE         16
    #define PLANNER_JUNCTION_DEVIATION  0.05f
    #define PLANNER_MIN_SPEED           0.1f
#endif

/*============================================================================
 *                    BACKLASH CONFIGURATION
 *============================================================================*/

#if CNC_USE_BACKLASH
    #define DEFAULT_BACKLASH_X          0.0f
    #define DEFAULT_BACKLASH_Y          0.0f
    #define DEFAULT_BACKLASH_Z          0.0f
    #define DEFAULT_BACKLASH_V          0.0f
    #define DEFAULT_BACKLASH_ENABLED    0
#endif

/*============================================================================
 *                    INVERT MASKS
 *                    bit0=X, bit1=Y, bit2=Z, bit3=V
 *============================================================================*/

/** Step pulse invert: 0=active HIGH, bit set=active LOW */
#define DEFAULT_STEP_INVERT_MASK        0b0000

/**
 * Direction invert:
 *   bit2 = Z inverted (hardware: PB7, invert_dir=1)
 *   bit3 = V inverted (hardware: PB8, invert_dir=1)
 *   X, Y not inverted
 */
#define DEFAULT_DIR_INVERT_MASK         0b1100

/** Limit switch invert: 0=active LOW (NC), bit set=active HIGH (NO) */
#define DEFAULT_LIMIT_INVERT_MASK       0b0000

/** Enable pin invert: 0=active LOW, 1=active HIGH */
#define DEFAULT_ENABLE_INVERT           0

/*============================================================================
 *                    ACCELERATION CONFIGURATION
 *============================================================================*/

#if CNC_USE_ACCELERATION

    #define DEFAULT_ACCELERATION_X      100.0f
    #define DEFAULT_ACCELERATION_Y      100.0f
    #define DEFAULT_ACCELERATION_Z      50.0f
    #define DEFAULT_ACCELERATION_V      50.0f

    /** Minimum step frequency (Hz) - start/end speed */
    #define CNC_MIN_STEP_FREQ           100

    /**
     * Time-based accel update interval (ms)
     *   2 = update every 2ms (good balance for STM32F1)
     */
    #define CNC_ACCEL_UPDATE_MS         2U

#endif

/*============================================================================
 *                    HOMING CONFIGURATION
 *============================================================================*/

#if CNC_USE_HOMING

    #define DEFAULT_HOMING_ENABLE       1

    /**
     * Homing direction mask (bit=1: negative direction)
     *   0b1111 = all axes home to negative direction
     */
    #define DEFAULT_HOMING_DIR_MASK     0b1111

    /**
     * Homing axis mask (bit=1: axis will be homed)
     *   0b0111 = home X, Y, Z only (V has no limit switch by default)
     */
    #define DEFAULT_HOMING_AXIS_MASK    0b0111

    #define DEFAULT_HOMING_SEEK_RATE    500.0f      /* mm/min */
    #define DEFAULT_HOMING_FEED_RATE    25.0f       /* mm/min */
    #define DEFAULT_HOMING_PULLOFF      1.0f        /* mm */
    #define DEFAULT_HOMING_DEBOUNCE_MS  25
    #define DEFAULT_HOMING_CYCLE        1           /* 1 = Z first, then X, then Y */

#endif

/*============================================================================
 *                    RELAY / SPINDLE / COOLANT
 *============================================================================*/

#if CNC_USE_RELAY

    #define RELAY_SPINDLE_PORT          GPIOB
    #define RELAY_SPINDLE_PIN           13

    #define RELAY_COOLANT_MIST_PORT     GPIOB
    #define RELAY_COOLANT_MIST_PIN      14

    #define RELAY_COOLANT_FLOOD_PORT    GPIOB
    #define RELAY_COOLANT_FLOOD_PIN     15

    #define RELAY_ACTIVE_HIGH           1

#endif

/*============================================================================
 *                    LIMIT SWITCH CONFIGURATION
 *============================================================================*/

#if CNC_USE_LIMIT_SWITCH

    #define CNC_LIMIT_ACTIVE_HIGH       0

    #define LIMIT_PORT                  GPIOA
    #define LIMIT_X_PIN                 3
    #define LIMIT_Y_PIN                 4
    #define LIMIT_Z_PIN                 5

    /** V-axis limit switch - set LIMIT_V_ENABLED=1 if hardware present */
    #define LIMIT_V_PIN                 6
    #define LIMIT_V_ENABLED             0

#endif

/*============================================================================
 *                    AUTO REPORT
 *============================================================================*/

#if CNC_AUTO_REPORT_ENABLE
    #define CNC_AUTO_REPORT_INTERVAL_MS 300
#endif

/*============================================================================
 *                    SOFT LIMIT
 *============================================================================*/

#define CNC_SOFT_LIMIT_DEFAULT          1
#define CNC_ALLOW_NEGATIVE_COORD        1

/*============================================================================
 *                    DEFAULT MOTION PARAMETERS
 *============================================================================*/

#define DEFAULT_STEPS_PER_MM_X          1000.0f
#define DEFAULT_STEPS_PER_MM_Y          1000.0f
#define DEFAULT_STEPS_PER_MM_Z          1000.0f
#define DEFAULT_STEPS_PER_MM_V          1000.0f

#define DEFAULT_MAX_FEEDRATE_X          3000.0f
#define DEFAULT_MAX_FEEDRATE_Y          3000.0f
#define DEFAULT_MAX_FEEDRATE_Z          500.0f
#define DEFAULT_MAX_FEEDRATE_V          500.0f
#define DEFAULT_FEEDRATE                1000.0f

#define DEFAULT_MAX_TRAVEL_X            20000.0f
#define DEFAULT_MAX_TRAVEL_Y            20000.0f
#define DEFAULT_MAX_TRAVEL_Z            20000.0f
#define DEFAULT_MAX_TRAVEL_V            20000.0f

#define MIN_STEPS_PER_MM                1.0f
#define MAX_STEPS_PER_MM                100000.0f
#define MIN_TIMER_PERIOD_US             2
#define MAX_TIMER_PERIOD_US             65535
#define MIN_STEP_FREQ_SAFE              10

#define UART_BUFFER_SIZE                256
#define GCODE_LINE_SIZE                 128

/*============================================================================
 *                    PIN DEFINITIONS
 *
 *  Step pins (bit-bang on TIM output pins used as GPIO):
 *    X : PA1  (TIM2_CH2 pin)
 *    Y : PA0  (TIM2_CH1 pin)
 *    Z : PA6  (TIM3_CH1 pin)
 *    V : PB0  (TIM3_CH3 pin)
 *
 *  Dir pins:
 *    X : PB5  (invert_dir = 0)
 *    Y : PB6  (invert_dir = 0)
 *    Z : PB7  (invert_dir = 1  → DEFAULT_DIR_INVERT_MASK bit2)
 *    V : PB8  (invert_dir = 1  → DEFAULT_DIR_INVERT_MASK bit3)
 *============================================================================*/

/* Step pins */
#define STEP_PORT_X                     GPIOA
#define STEP_X_PIN                      1

#define STEP_PORT_Y                     GPIOA
#define STEP_Y_PIN                      0

#define STEP_PORT_Z                     GPIOA
#define STEP_Z_PIN                      6

#define STEP_PORT_V                     GPIOB
#define STEP_V_PIN                      0

/* Dir pins - each axis has its own port macro for CRL/CRH access */
#define DIR_PORT_X                      GPIOB
#define DIR_X_PIN                       5

#define DIR_PORT_Y                      GPIOB
#define DIR_Y_PIN                       6

#define DIR_PORT_Z                      GPIOB
#define DIR_Z_PIN                       7

#define DIR_PORT_V                      GPIOB
#define DIR_V_PIN                       8

/* Enable pins (optional) */
#if CNC_USE_ENABLE_PIN
    #define EN_PORT                     GPIOB
    #define EN_X_PIN                    10
    #define EN_Y_PIN                    11
    #define EN_Z_PIN                    12
    #define EN_V_PIN                    9
#endif

/*============================================================================
 *                    GUI PROTOCOL CONFIGURATION
 *============================================================================*/

#if CNC_USE_GUI_PROTOCOL

    #define GUI_NUM_PRESETS             4           /* A, B, C, D */
    #define GUI_SENTINEL_PULSE          (-9999)     /* axis inactive */
    #define GUI_SENTINEL_MM             (-9.999f)
    #define GUI_PULSE_PER_MM            1000.0f

    /*--- Default presets (mm), sentinel = axis does not move on GOTO ---*/

    /* Preset A */
    #define GUI_PRESET_A_X_MM           20.0f
    #define GUI_PRESET_A_Y_MM           2.0f
    #define GUI_PRESET_A_Z_PULSE        GUI_SENTINEL_PULSE
    #define GUI_PRESET_A_V_PULSE        GUI_SENTINEL_PULSE

    /* Preset B */
    #define GUI_PRESET_B_X_MM           2.0f
    #define GUI_PRESET_B_Y_MM           1.0f
    #define GUI_PRESET_B_Z_PULSE        GUI_SENTINEL_PULSE
    #define GUI_PRESET_B_V_PULSE        GUI_SENTINEL_PULSE

    /* Preset C */
    #define GUI_PRESET_C_X_MM           20.0f
    #define GUI_PRESET_C_Y_MM           1.0f
    #define GUI_PRESET_C_Z_PULSE        GUI_SENTINEL_PULSE
    #define GUI_PRESET_C_V_PULSE        GUI_SENTINEL_PULSE

    /* Preset D */
    #define GUI_PRESET_D_X_MM           1.0f
    #define GUI_PRESET_D_Y_MM           0.010f
    #define GUI_PRESET_D_Z_PULSE        GUI_SENTINEL_PULSE
    #define GUI_PRESET_D_V_PULSE        GUI_SENTINEL_PULSE

#endif /* CNC_USE_GUI_PROTOCOL */

/*============================================================================
 *                    TIMER AUTO CONFIG
 *============================================================================*/

#if (CNC_TIMER_SELECT == 2)
    #define CNC_TIMER               TIM2
    #define CNC_TIMER_IRQn          TIM2_IRQn
    #define CNC_TIMER_IRQHandler    TIM2_IRQHandler
    #define CNC_TIMER_RCC           RCC_APB1ENR_TIM2EN
#elif (CNC_TIMER_SELECT == 3)
    #define CNC_TIMER               TIM3
    #define CNC_TIMER_IRQn          TIM3_IRQn
    #define CNC_TIMER_IRQHandler    TIM3_IRQHandler
    #define CNC_TIMER_RCC           RCC_APB1ENR_TIM3EN
#elif (CNC_TIMER_SELECT == 4)
    #define CNC_TIMER               TIM4
    #define CNC_TIMER_IRQn          TIM4_IRQn
    #define CNC_TIMER_IRQHandler    TIM4_IRQHandler
    #define CNC_TIMER_RCC           RCC_APB1ENR_TIM4EN
#endif

/*============================================================================
 *                    UART AUTO CONFIG
 *============================================================================*/

#if (CNC_UART_SELECT == 1)
    #define CNC_UART                USART1
    #define CNC_UART_IRQn           USART1_IRQn
    #define CNC_UART_IRQHandler     USART1_IRQHandler
    #define CNC_UART_RCC            RCC_APB2ENR_USART1EN
    #define CNC_UART_PORT           GPIOA
    #define CNC_UART_APB2           1
#elif (CNC_UART_SELECT == 2)
    #define CNC_UART                USART2
    #define CNC_UART_IRQn           USART2_IRQn
    #define CNC_UART_IRQHandler     USART2_IRQHandler
    #define CNC_UART_RCC            RCC_APB1ENR_USART2EN
    #define CNC_UART_PORT           GPIOA
    #define CNC_UART_APB2           0
#elif (CNC_UART_SELECT == 3)
    #define CNC_UART                USART3
    #define CNC_UART_IRQn           USART3_IRQn
    #define CNC_UART_IRQHandler     USART3_IRQHandler
    #define CNC_UART_RCC            RCC_APB1ENR_USART3EN
    #define CNC_UART_PORT           GPIOB
    #define CNC_UART_APB2           0
#endif

/*============================================================================
 *                    STEP PIN MACROS
 *============================================================================*/

#define _PIN_MASK(pin)              (1U << (pin))

#define STEP_X_HIGH()               (STEP_PORT_X->BSRR = _PIN_MASK(STEP_X_PIN))
#define STEP_X_LOW()                (STEP_PORT_X->BRR  = _PIN_MASK(STEP_X_PIN))
#define STEP_Y_HIGH()               (STEP_PORT_Y->BSRR = _PIN_MASK(STEP_Y_PIN))
#define STEP_Y_LOW()                (STEP_PORT_Y->BRR  = _PIN_MASK(STEP_Y_PIN))
#define STEP_Z_HIGH()               (STEP_PORT_Z->BSRR = _PIN_MASK(STEP_Z_PIN))
#define STEP_Z_LOW()                (STEP_PORT_Z->BRR  = _PIN_MASK(STEP_Z_PIN))
#define STEP_V_HIGH()               (STEP_PORT_V->BSRR = _PIN_MASK(STEP_V_PIN))
#define STEP_V_LOW()                (STEP_PORT_V->BRR  = _PIN_MASK(STEP_V_PIN))

/*============================================================================
 *                    TYPES - MACHINE STATE
 *============================================================================*/

typedef enum {
    CNC_STATE_IDLE = 0,
    CNC_STATE_RUN,
    CNC_STATE_HOLD,
    CNC_STATE_HOMING,
    CNC_STATE_ALARM,
    CNC_STATE_LIMIT_TRIGGERED,
    CNC_STATE_GOWDELAY,
    CNC_STATE_GOWDELAY_RECT,
    CNC_STATE_GOTO           /* GOTO preset motion */
} CNC_State_t;

typedef enum {
    CNC_COORD_ABSOLUTE = 0,
    CNC_COORD_INCREMENTAL
} CNC_CoordMode_t;

typedef enum {
    CNC_UNIT_MM = 0,
    CNC_UNIT_INCH
} CNC_Unit_t;

/*============================================================================
 *                    TYPES - GUI PROTOCOL
 *============================================================================*/

#if CNC_USE_GUI_PROTOCOL

typedef enum {
    GUI_ACTION_IDLE = 0,
    GUI_ACTION_STOP,
    GUI_ACTION_GOTO_A,
    GUI_ACTION_GOTO_B,
    GUI_ACTION_GOTO_C,
    GUI_ACTION_GOTO_D,
    GUI_ACTION_GCODE_MOVE,
} GUI_Action_t;

/** 4-axis preset position in pulses */
typedef struct {
    int32_t axis[CNC_NUM_AXES];  /* GUI_SENTINEL_PULSE = axis inactive */
} GUI_Position_t;

typedef struct {
    GUI_Action_t    current_action;
    GUI_Position_t  presets[GUI_NUM_PRESETS];  /* 0=A, 1=B, 2=C, 3=D */
    bool            initialized;
} GUI_Protocol_t;

#endif /* CNC_USE_GUI_PROTOCOL */

/*============================================================================
 *                    TYPES - DMA UART
 *============================================================================*/

#if CNC_USE_DMA_UART

typedef struct {
    uint8_t          buffer[UART_DMA_BUFFER_SIZE];
    volatile uint16_t write_pos;
    volatile uint16_t read_pos;
} CNC_DMA_Buffer_t;

#endif

/*============================================================================
 *                    TYPES - COMMAND QUEUE
 *============================================================================*/

#if CNC_USE_COMMAND_QUEUE

typedef struct {
    char             commands[CMD_QUEUE_SIZE][CMD_MAX_LENGTH];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} CNC_CommandQueue_t;

#endif

/*============================================================================
 *                    TYPES - MOTION PLANNER
 *============================================================================*/

#if CNC_USE_PLANNER

typedef struct {
    float   target[CNC_NUM_AXES];
    float   feedrate;
    bool    rapid;

    float   distance;
    float   unit_vec[CNC_NUM_AXES];
    float   entry_speed_sqr;
    float   max_entry_speed_sqr;
    float   exit_speed_sqr;
    float   max_exit_speed_sqr;
    float   nominal_speed;
    float   nominal_speed_sqr;
    float   acceleration;

    uint8_t recalculate_flag;
    uint8_t nominal_length_flag;
} PlannerBlock_t;

typedef struct {
    PlannerBlock_t   blocks[PLANNER_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;

    float   previous_unit_vec[CNC_NUM_AXES];
    float   previous_nominal_speed;
} MotionPlanner_t;

#endif

/*============================================================================
 *                    TYPES - BACKLASH
 *============================================================================*/

#if CNC_USE_BACKLASH

typedef struct {
    float   backlash[CNC_NUM_AXES];
    int8_t  last_direction[CNC_NUM_AXES];
    bool    enabled;
} BacklashComp_t;

#endif

/*============================================================================
 *                    TYPES - HOMING
 *============================================================================*/

#if CNC_USE_HOMING

typedef enum {
    HOMING_IDLE = 0,
    HOMING_SEEK,
    HOMING_PULLOFF1,
    HOMING_FEED,
    HOMING_PULLOFF2,
    HOMING_COMPLETE
} HomingPhase_t;

#endif

/*============================================================================
 *                    TYPES - ACCELERATION
 *============================================================================*/

#if CNC_USE_ACCELERATION

typedef enum {
    ACCEL_PHASE_ACCEL = 0,
    ACCEL_PHASE_CRUISE,
    ACCEL_PHASE_DECEL
} AccelPhase_t;

typedef struct {
    AccelPhase_t phase;
    float current_speed;        /* mm/s */
    float target_speed;         /* mm/s */
    float entry_speed;          /* mm/s */
    float exit_speed;           /* mm/s */
    float accel_rate;           /* mm/s² */

    uint32_t accel_steps;
    uint32_t cruise_steps;
    uint32_t decel_steps;
    uint32_t decel_start;

    float mm_per_step;
    float spm_dominant;         /* steps/mm of dominant axis */
    float inv_spm_dominant;     /* 1 / spm_dominant */

    /* Time-base: updated from main context */
    volatile uint32_t last_update_tick;
    volatile bool     needs_recalc;

    /* Precomputed for ISR (no float in ISR) */
    volatile uint32_t precomp_high_ticks;
    volatile uint32_t precomp_low_ticks;
    volatile uint32_t precomp_period;
} AccelState_t;

#endif

/*============================================================================
 *                    TYPES - GOWDELAY
 *============================================================================*/

typedef enum {
    GOWDELAY_IDLE = 0,
    GOWDELAY_MOVING,
    GOWDELAY_WAITING,
    GOWDELAY_COMPLETE
} GoWDelayPhase_t;

typedef struct {
    GoWDelayPhase_t phase;
    float   start_pos[CNC_NUM_AXES];
    float   target_pos[CNC_NUM_AXES];
    float   unit_vector[CNC_NUM_AXES];
    float   total_distance;
    float   step_distance;
    float   distance_moved;
    uint32_t delay_ms;
    float   feedrate;
    uint32_t current_step;
    uint32_t total_steps;
    uint32_t delay_start_tick;
} GoWDelayState_t;

/*============================================================================
 *                    TYPES - GOWDELAYRECT
 *============================================================================*/

typedef enum {
    GWDRECT_IDLE = 0,
    GWDRECT_MOVE_X,
    GWDRECT_WAIT_X,
    GWDRECT_MOVE_Y,
    GWDRECT_WAIT_Y,
    GWDRECT_COMPLETE
} GoWDelayRectPhase_t;

typedef struct {
    GoWDelayRectPhase_t phase;

    float   start_pos[CNC_NUM_AXES];
    float   rect_size_x;
    float   rect_size_y;

    float   step_distance;
    uint32_t delay_ms;
    float   feedrate;

    float   current_x;
    float   current_y;
    int8_t  x_direction;

    uint32_t current_row;
    uint32_t total_rows;
    uint32_t current_step_in_row;
    uint32_t steps_per_row;
    uint32_t total_steps;
    uint32_t completed_steps;

    uint32_t delay_start_tick;
} GoWDelayRectState_t;

/*============================================================================
 *                    TYPES - CONFIG
 *============================================================================*/

typedef struct {
    /* Motion */
    float   steps_per_mm[CNC_NUM_AXES];
    float   max_feedrate[CNC_NUM_AXES];
    float   max_travel[CNC_NUM_AXES];
    float   acceleration[CNC_NUM_AXES];

    /* Invert masks (bit0=X, bit1=Y, bit2=Z, bit3=V) */
    uint8_t step_invert_mask;
    uint8_t dir_invert_mask;
    uint8_t limit_invert_mask;
    uint8_t enable_invert;

    /* Limits */
    bool    soft_limit_enabled;
    bool    allow_negative;
    bool    hard_limit_enabled;

    /* Homing */
    bool    homing_enabled;
    uint8_t homing_dir_mask;    /* bit=1: home negative direction */
    uint8_t homing_axis_mask;   /* bit=1: axis will be homed */
    float   homing_seek_rate;   /* mm/min */
    float   homing_feed_rate;   /* mm/min */
    float   homing_pulloff;     /* mm */
    uint8_t homing_cycle;       /* 0=together, 1=Z-X-Y, 2=Z-XY */
    uint16_t homing_debounce_ms;

    /* Backlash */
#if CNC_USE_BACKLASH
    float   backlash[CNC_NUM_AXES];
    bool    backlash_enabled;
#endif

    /* Auto report */
    uint32_t auto_report_interval;  /* ms, 0=off */
} CNC_Config_t;

/*============================================================================
 *                    TYPES - MACHINE
 *============================================================================*/

typedef struct {
    CNC_State_t     state;

    float           position[CNC_NUM_AXES];
    float           target[CNC_NUM_AXES];

    volatile int32_t step_count[CNC_NUM_AXES];
    int32_t         steps_total[CNC_NUM_AXES];
    int8_t          direction[CNC_NUM_AXES];

    float           feedrate;
    CNC_CoordMode_t coord_mode;
    CNC_Unit_t      unit;

    volatile bool   is_moving;
    bool            motors_enabled;
    bool            limit_triggered[CNC_NUM_AXES];

    /* Relay states */
    bool            spindle_on;
    bool            coolant_mist_on;
    bool            coolant_flood_on;

    /* Homing */
#if CNC_USE_HOMING
    HomingPhase_t   homing_phase;
    uint8_t         homing_axis;
    uint8_t         homing_cycle_step;
    uint8_t         homing_axes_pending;
    bool            homed[CNC_NUM_AXES];
#endif

    /* Acceleration */
#if CNC_USE_ACCELERATION
    AccelState_t    accel;
#endif

    /* GOWDELAY */
    GoWDelayState_t     gowdelay;
    GoWDelayRectState_t gowdelay_rect;

    /* Backlash helpers */
    float           logical_delta[CNC_NUM_AXES];
    int32_t         logical_steps_total[CNC_NUM_AXES];

    uint8_t         dominant_axis;
    float           move_distance;

    uint32_t        last_report_time;

    /* GUI Protocol */
#if CNC_USE_GUI_PROTOCOL
    GUI_Action_t    gui_action;
#endif
} CNC_Machine_t;

/*============================================================================
 *                    TYPES - UART RING BUFFER
 *============================================================================*/

typedef struct {
    uint8_t          buffer[UART_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} CNC_RingBuffer_t;

/*============================================================================
 *                    GLOBALS
 *============================================================================*/

extern CNC_Machine_t     cnc_machine;
extern CNC_Config_t      cnc_config;
extern CNC_RingBuffer_t  cnc_uart_rx;
extern volatile uint32_t cnc_systick_ms;
extern volatile bool     cnc_send_ok_flag;
extern volatile bool     cnc_auto_report_flag;

#if CNC_USE_GUI_PROTOCOL
extern GUI_Protocol_t    gui_proto;
#endif

/*============================================================================
 *                    INIT
 *============================================================================*/

void CNC_Init(void);
void CNC_SystemClock_Config(void);
void CNC_LoadDefaultConfig(void);

/*============================================================================
 *                    UART - BASIC
 *============================================================================*/

void CNC_UART_PutChar(char c);
void CNC_UART_PutString(const char *str);
void CNC_UART_PutInt(int32_t num);
void CNC_UART_PutFloat(float num, uint8_t decimals);
bool CNC_UART_Available(void);
char CNC_UART_GetChar(void);
bool CNC_UART_GetLine(char *line, uint16_t max_len);
void CNC_UART_SendOK(void);
void CNC_UART_SendError(uint8_t code);

/*============================================================================
 *                    UART - DMA
 *============================================================================*/

#if CNC_USE_DMA_UART
void     CNC_UART_DMA_Init(void);
uint16_t CNC_UART_DMA_Available(void);
uint8_t  CNC_UART_DMA_Read(void);
void     CNC_UART_DMA_Flush(void);
#endif

/*============================================================================
 *                    COMMAND QUEUE
 *============================================================================*/

#if CNC_USE_COMMAND_QUEUE
void    CNC_Queue_Init(void);
bool    CNC_Queue_Push(const char *cmd);
bool    CNC_Queue_Pop(char *cmd);
bool    CNC_Queue_IsFull(void);
bool    CNC_Queue_IsEmpty(void);
uint8_t CNC_Queue_Count(void);
uint8_t CNC_Queue_Available(void);
void    CNC_Queue_Clear(void);
#endif

/*============================================================================
 *                    MOTION PLANNER
 *============================================================================*/

#if CNC_USE_PLANNER
void            CNC_Planner_Init(void);
bool            CNC_Planner_AddBlock(float *target, float feedrate, bool rapid);
PlannerBlock_t* CNC_Planner_GetCurrentBlock(void);
void            CNC_Planner_DiscardCurrentBlock(void);
bool            CNC_Planner_IsBufferEmpty(void);
bool            CNC_Planner_IsBufferFull(void);
uint8_t         CNC_Planner_BlocksAvailable(void);
uint8_t         CNC_Planner_BlocksCount(void);
void            CNC_Planner_Recalculate(void);
void            CNC_Planner_Clear(void);
#endif

/*============================================================================
 *                    BACKLASH COMPENSATION
 *============================================================================*/

#if CNC_USE_BACKLASH
void  CNC_Backlash_Init(void);
void  CNC_Backlash_SetValue(uint8_t axis, float value);
float CNC_Backlash_GetValue(uint8_t axis);
void  CNC_Backlash_Enable(bool enable);
bool  CNC_Backlash_IsEnabled(void);
void  CNC_Backlash_Compensate(uint8_t axis, int8_t new_direction, float *delta);
void  CNC_Backlash_ResetDirections(void);
#endif

/*============================================================================
 *                    STEP / DIRECTION CONTROL
 *============================================================================*/

void CNC_Step_Pulse(uint8_t axis, bool active);
void CNC_Step_AllLow(void);
void CNC_SetDirection(uint8_t axis, int8_t dir);
void CNC_EnableMotors(bool enable);

/*============================================================================
 *                    MOTION
 *============================================================================*/

void CNC_Move(float *target, float feedrate, bool rapid);
void CNC_Move_NoAccel(float *target, float feedrate);
void CNC_MoveWithCompensation(float *target, float feedrate, bool rapid);
bool CNC_IsMoving(void);
void CNC_Stop(void);
void CNC_WaitComplete(void);
void CNC_SetPosition(float *pos);
void CNC_SetAxisPosition(uint8_t axis, float value);
void CNC_GetPosition(float *pos);

/*============================================================================
 *                    ACCELERATION
 *============================================================================*/

#if CNC_USE_ACCELERATION
void     CNC_Accel_Plan(float distance, float feedrate, float accel);
void     CNC_Accel_PlanWithEntryExit(float distance, float feedrate, float accel,
                                     float entry_speed, float exit_speed);
void     CNC_Accel_Update(void);
void     CNC_Accel_TimerTick(void);
uint32_t CNC_Accel_GetCurrentFreq(void);
float    CNC_Accel_GetCurrentSpeed(void);
#endif

/*============================================================================
 *                    HOMING
 *============================================================================*/

#if CNC_USE_HOMING
void CNC_HomeAll(void);
void CNC_HomeAxis(uint8_t axis);
void CNC_Homing_Process(void);
bool CNC_Homing_IsActive(void);
#endif

/*============================================================================
 *                    LIMITS
 *============================================================================*/

#if CNC_USE_LIMIT_SWITCH
void CNC_Limit_Init(void);
bool CNC_Limit_IsTriggered(uint8_t axis);
bool CNC_Limit_CheckAll(void);
void CNC_Limit_ClearAlarm(void);
#endif

/*============================================================================
 *                    RELAY / SPINDLE / COOLANT
 *============================================================================*/

#if CNC_USE_RELAY
void CNC_Relay_Init(void);
void CNC_Spindle_On(void);
void CNC_Spindle_Off(void);
void CNC_Coolant_Mist_On(void);
void CNC_Coolant_Mist_Off(void);
void CNC_Coolant_Flood_On(void);
void CNC_Coolant_Flood_Off(void);
void CNC_Coolant_Off(void);
#endif

/*============================================================================
 *                    CONFIG
 *============================================================================*/

void CNC_Config_Show(void);
void CNC_Config_Set(const char *param, float value);
void CNC_Config_SetStepsPerMM(uint8_t axis, float value);
void CNC_Config_SetMaxFeedrate(uint8_t axis, float value);
void CNC_Config_SetMaxTravel(uint8_t axis, float value);
void CNC_Config_SetAcceleration(uint8_t axis, float value);
void CNC_Config_SetSoftLimit(bool enable);
void CNC_Config_SetAutoReport(uint32_t interval_ms);

/*============================================================================
 *                    G-CODE / COMMANDS
 *============================================================================*/

uint8_t CNC_Execute(const char *line);
void    CNC_ProcessCommand(const char *cmd);
void    CNC_SendStatus(void);
void    CNC_SendStatusCompact(void);

/*============================================================================
 *                    GUI PROTOCOL API
 *============================================================================*/

#if CNC_USE_GUI_PROTOCOL

void        CNC_GUI_Init(void);
void        CNC_GUI_Handle_STOP(void);
void        CNC_GUI_Handle_SET(const char *params);
void        CNC_GUI_Handle_GOTO(const char *params);
void        CNC_GUI_Handle_RETURN(const char *params);
void        CNC_GUI_SendStatus(void);
const char* CNC_GUI_GetStateString(void);

#endif /* CNC_USE_GUI_PROTOCOL */

/*============================================================================
 *                    GOWDELAY
 *============================================================================*/

void CNC_GoWDelay_Start(float *target, float step_distance,
                        uint32_t delay_ms, float feedrate);
void CNC_GoWDelay_Process(void);
void CNC_GoWDelay_Stop(void);
bool CNC_GoWDelay_IsActive(void);

/*============================================================================
 *                    GOWDELAYRECT
 *============================================================================*/

void CNC_GoWDelayRect_Start(float size_x, float size_y, float step_distance,
                            uint32_t delay_ms, float feedrate);
void CNC_GoWDelayRect_Process(void);
void CNC_GoWDelayRect_Stop(void);
bool CNC_GoWDelayRect_IsActive(void);

/*============================================================================
 *                    MAIN LOOP
 *============================================================================*/

void     CNC_Process(void);
uint32_t CNC_GetTick(void);

/*============================================================================
 *                    UTILITY
 *============================================================================*/

float CNC_min(float a, float b);
float CNC_max(float a, float b);
float CNC_clamp(float value, float min_val, float max_val);

/*============================================================================
 *                    AXIS NAME HELPER
 *============================================================================*/

static inline char CNC_AxisName(uint8_t axis) {
    switch (axis) {
        case 0:  return 'X';
        case 1:  return 'Y';
        case 2:  return 'Z';
        case 3:  return 'V';
        default: return '?';
    }
}

/*============================================================================
 *                    $ PARAMETER REFERENCE
 *============================================================================*/
/*
 * Invert Masks:
 *   $2  = Step Invert Mask      (0-15, bit0=X, bit1=Y, bit2=Z, bit3=V)
 *   $3  = Dir Invert Mask       (0-15, default=12 → Z,V inverted)
 *   $4  = Enable Invert         (0/1)
 *   $5  = Limit Invert Mask     (0-15)
 *
 * Limits:
 *   $20 = Soft Limit Enable     (0/1)
 *   $21 = Hard Limit Enable     (0/1)
 *
 * Homing:
 *   $22 = Homing Enable         (0/1)
 *   $23 = Homing Dir Mask       (0-15, bit=1: negative)
 *   $24 = Homing Feed Rate      (mm/min)
 *   $25 = Homing Seek Rate      (mm/min)
 *   $26 = Homing Axis Mask      (0-15, bit=1: axis homed)
 *   $27 = Homing Pulloff        (mm)
 *   $28 = Homing Cycle          (0/1/2)
 *
 * Auto Report:
 *   $32 = Auto Report Interval  (ms, 0=off)
 *
 * Steps/mm:
 *   $100 = X Steps/mm
 *   $101 = Y Steps/mm
 *   $102 = Z Steps/mm
 *   $103 = V Steps/mm
 *
 * Max Feedrate (mm/min):
 *   $110 = X Max Rate
 *   $111 = Y Max Rate
 *   $112 = Z Max Rate
 *   $113 = V Max Rate
 *
 * Acceleration (mm/s²):
 *   $120 = X Accel
 *   $121 = Y Accel
 *   $122 = Z Accel
 *   $123 = V Accel
 *
 * Max Travel (mm):
 *   $130 = X Max Travel
 *   $131 = Y Max Travel
 *   $132 = Z Max Travel
 *   $133 = V Max Travel
 */

/*============================================================================
 *                    COMMAND REFERENCE
 *============================================================================*/
/*
 * GUI Protocol (case-sensitive):
 *   STOP                     - Emergency stop (accepted when BUSY)
 *   GOTO A/B/C/D             - Move to preset position
 *   SET A [X<mm>][Y<mm>]...  - Update preset
 *   SET X/Y/Z/V              - Reset axis position to 0
 *   RETURN A/B/C/D           - Query preset coordinates
 *
 * G-code:
 *   G0 X_ Y_ Z_ V_           - Rapid move
 *   G1 X_ Y_ Z_ V_ F_        - Linear move (F in mm/min)
 *   G20                      - Inch mode
 *   G21                      - MM mode
 *   G28                      - Home all axes
 *   G90                      - Absolute coordinates
 *   G91                      - Incremental coordinates
 *   G92 X_ Y_ Z_ V_          - Set position
 *
 * M-code:
 *   M0/M2/M30                - Program stop
 *   M3                       - Spindle ON
 *   M5                       - Spindle OFF
 *   M7                       - Coolant Mist ON
 *   M8                       - Coolant Flood ON
 *   M9                       - Coolant OFF
 *   M17                      - Enable motors
 *   M18/M84                  - Disable motors
 *   M114                     - Report position
 *
 * Custom Commands:
 *   STOP / !                 - Emergency stop
 *   PAUSE                    - Pause motion
 *   RESUME                   - Resume motion
 *   RESET                    - Full reset
 *   HOME                     - Home all axes
 *   HOMEX/HOMEY/HOMEZ/HOMEV  - Home single axis
 *   ENABLE / DISABLE         - Motor enable/disable
 *   LIMITON / LIMITOFF       - Soft limit on/off
 *   LIMITS                   - Show limit switch status
 *   CLEAR                    - Clear alarm
 *   STATUS / ?               - Show status
 *   $$ / CONFIG              - Show config
 *   $xxx=value               - Set parameter
 *   REPORT <ms>              - Set auto report interval
 *   REPORT                   - Turn off auto report
 *   HELP                     - Show help
 *   QUEUE                    - Show command queue status
 *   PLANNER                  - Show motion planner status
 *   CLEARQUEUE               - Clear command queue
 *   CLEARPLANNER             - Clear motion planner
 *   SPINON/SPINOFF           - Spindle control
 *   MISTON/MISTOFF           - Coolant mist control
 *   FLOODON/FLOODOFF         - Coolant flood control
 *   COOLOFF                  - All coolant off
 *
 * GOWDELAY X_ Y_ Z_ V_ D_ W_ [F_]
 *   D = step distance (mm)
 *   W = wait time per step (ms)
 *   F = feedrate (mm/min, optional)
 *
 * GOWDELAYRECT X_ Y_ D_ W_ [F_]
 *   X = rectangle width (mm, can be negative)
 *   Y = rectangle height (mm, can be negative)
 *   D = step distance (mm)
 *   W = wait time per step (ms)
 *
 * Status format (GUI Protocol):
 *   <STATE|WPos:X.XXX,Y.YYY,Z.ZZZ,V.VVV>
 *
 * STATE strings:
 *   Idle         - Machine idle
 *   Run GCode    - Executing G-code
 *   GoTo A/B/C/D - Moving to preset
 *   STOP         - After STOP command
 *   Homing       - Homing cycle active
 *   Hold         - Motion paused
 *
 * Error Codes:
 *   1 = G-code parse error
 *   2 = Unknown G-code
 *   3 = Unknown M-code
 *   4 = Invalid parameter
 *   5 = Invalid feedrate
 *   6 = Soft limit exceeded
 *   7 = Homing required
 *   8 = Alarm state
 *   9 = Queue/Planner full
 *   10 = Invalid command
 */

/*============================================================================
 *                    VERSION INFO
 *============================================================================*/

#define CNC_VERSION_MAJOR           1
#define CNC_VERSION_MINOR           0
#define CNC_VERSION_PATCH           0
#define CNC_VERSION_STRING          "1.0.0"

/*============================================================================
 *                    FEATURE FLAGS
 *============================================================================*/

#define CNC_FEATURE_DMA_UART        (CNC_USE_DMA_UART)
#define CNC_FEATURE_CMD_QUEUE       (CNC_USE_COMMAND_QUEUE)
#define CNC_FEATURE_PLANNER         (CNC_USE_PLANNER)
#define CNC_FEATURE_BACKLASH        (CNC_USE_BACKLASH)
#define CNC_FEATURE_ACCEL           (CNC_USE_ACCELERATION)
#define CNC_FEATURE_HOMING          (CNC_USE_HOMING)
#define CNC_FEATURE_LIMIT           (CNC_USE_LIMIT_SWITCH)
#define CNC_FEATURE_RELAY           (CNC_USE_RELAY)
#define CNC_FEATURE_GUI             (CNC_USE_GUI_PROTOCOL)





#endif /* INC_GRBL_AXISARM_H_ */
