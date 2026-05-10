/*
 * grbl_AxisArm.c
 *
 *  Created on: May 5, 2026
 *      Author: DELL
 */

#include "grbl_AxisArm.h"

/*============================================================================
 *                    GLOBAL VARIABLES
 *============================================================================*/

CNC_Machine_t    cnc_machine;
CNC_Config_t     cnc_config;
CNC_RingBuffer_t cnc_uart_rx;

volatile uint32_t cnc_systick_ms   = 0;
volatile bool     cnc_send_ok_flag  = false;
volatile bool     cnc_auto_report_flag = false;

#if CNC_USE_GUI_PROTOCOL
GUI_Protocol_t gui_proto;
#endif

/*============================================================================
 *                    PRIVATE VARIABLES
 *============================================================================*/

/* Bresenham */
static volatile int64_t bresenham_total;
static volatile int64_t bresenham_counter[CNC_NUM_AXES];
static volatile int64_t bresenham_steps[CNC_NUM_AXES];
static volatile int64_t step_events_remaining;
#define MAX_STEPS_PER_MOVE      100000000LL

/* Parser */
static float current_feedrate = DEFAULT_FEEDRATE;
static bool  rapid_mode       = false;

/* Auto report */
static volatile uint32_t auto_report_counter = 0;

/* Acceleration */
#if CNC_USE_ACCELERATION
static volatile uint32_t current_step_period = 0;
#endif

/* Homing */
#if CNC_USE_HOMING
static volatile uint32_t homing_step_count = 0;
static volatile bool     homing_limit_hit  = false;
#endif

/* DMA UART */
#if CNC_USE_DMA_UART
static CNC_DMA_Buffer_t uart_dma_rx;
#endif

/* Command Queue */
#if CNC_USE_COMMAND_QUEUE
static CNC_CommandQueue_t cmd_queue;
#endif

/* Motion Planner */
#if CNC_USE_PLANNER
static MotionPlanner_t planner;
#endif

/* Backlash */
#if CNC_USE_BACKLASH
static BacklashComp_t backlash_comp;
#endif

/* Step pulse phase */
static volatile uint8_t step_phase     = 0;
static volatile uint32_t step_high_ticks = 0;
static volatile uint32_t step_low_ticks  = 0;
static volatile bool pending_step[CNC_NUM_AXES] = { false };

/*============================================================================
 *                    FORWARD DECLARATIONS
 *============================================================================*/

static void  CNC_GoWDelay_NextStep(void);
static void  CNC_GoWDelayRect_NextStepX(void);
static void  CNC_GoWDelayRect_NextStepY(void);
static bool  find_value(const char *line, char letter, float *value);
static bool  has_command(const char *line, char letter, int code);
static uint32_t CNC_Step_CalcDuty(uint32_t period_us);
static inline float CNC_SafeStepsPerMM(uint8_t axis);
static float CNC_ComputeVectorMaxSpeedMMs(const float *unit_vec);
static float CNC_ComputeVectorMaxAccelMMs2(const float *unit_vec);
static bool  CNC_IsPlannerStreamableLine(const char *line);
static bool  CNC_HandlePlannerStreamingInput(const char *line);
#if CNC_USE_COMMAND_QUEUE && CNC_USE_PLANNER
static void  CNC_ProcessQueuedStreamingCommands(void);
#endif

/*============================================================================
 *                    UTILITY
 *============================================================================*/

float CNC_min(float a, float b) { return (a < b) ? a : b; }
float CNC_max(float a, float b) { return (a > b) ? a : b; }
float CNC_clamp(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/*---------------------------------------------------------------------------*/
static float CNC_ComputeVectorMaxSpeedMMs(const float *unit_vec)
{
    float max_speed = 0.0f;
    bool  have_axis = false;
    if (!unit_vec) return DEFAULT_FEEDRATE / 60.0f;

    for (int i = 0; i < CNC_NUM_AXES; i++) {
        float u = fabsf(unit_vec[i]);
        if (u > 1e-6f) {
            float vs = (cnc_config.max_feedrate[i] / 60.0f) / u;
            if (!have_axis || vs < max_speed) { max_speed = vs; }
            have_axis = true;
        }
    }
    if (!have_axis) max_speed = DEFAULT_FEEDRATE / 60.0f;
    if (max_speed < 0.001f) max_speed = 0.001f;
    return max_speed;
}

static float CNC_ComputeVectorMaxAccelMMs2(const float *unit_vec)
{
    float max_accel = 0.0f;
    bool  have_axis = false;
    if (!unit_vec) return cnc_config.acceleration[0];

    for (int i = 0; i < CNC_NUM_AXES; i++) {
        float u = fabsf(unit_vec[i]);
        if (u > 1e-6f) {
            float va = cnc_config.acceleration[i] / u;
            if (!have_axis || va < max_accel) { max_accel = va; }
            have_axis = true;
        }
    }
    if (!have_axis) max_accel = cnc_config.acceleration[0];
    if (max_accel < 0.1f) max_accel = 0.1f;
    return max_accel;
}

static bool CNC_IsPlannerStreamableLine(const char *line)
{
#if !CNC_USE_PLANNER
    (void)line; return false;
#else
    if (!line || line[0] == '\0') return false;
    const char *p = line;
    bool saw_token = false;
    while (*p) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == ';' || *p == '(') break;
        char c = *p;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c=='N'||c=='X'||c=='Y'||c=='Z'||c=='V'||c=='F'||c=='G'||c=='M') {
            char *ep;
            float wv = strtof(p+1, &ep);
            if (ep == p+1) return false;
            if (*ep != '\0' && *ep != ' ' && *ep != '\t' &&
                *ep != ';'  && *ep != '(') return false;
            if (c == 'G') {
                int code = (int)wv;
                if (!(code==0||code==1||code==20||code==21||code==90||code==91))
                    return false;
            } else if (c == 'M') {
                int code = (int)wv;
                if (!(code==3||code==5||code==7||code==8||code==9||code==114))
                    return false;
            }
            saw_token = true;
            p = ep;
            continue;
        }
        return false;
    }
    return saw_token;
#endif
}

static bool CNC_HandlePlannerStreamingInput(const char *line)
{
#if !CNC_USE_PLANNER
    (void)line; return false;
#else
    if (!CNC_IsPlannerStreamableLine(line)) return false;
#if CNC_USE_COMMAND_QUEUE
    if (!CNC_Queue_IsEmpty() || CNC_Planner_IsBufferFull()) {
        if (!CNC_Queue_Push(line)) CNC_UART_SendError(9);
        else CNC_UART_SendOK();
        return true;
    }
#else
    if (CNC_Planner_IsBufferFull()) { CNC_UART_SendError(9); return true; }
#endif
    uint8_t error = CNC_Execute(line);
    if (error != 0) CNC_UART_SendError(error);
    else            CNC_UART_SendOK();
    return true;
#endif
}

#if CNC_USE_COMMAND_QUEUE && CNC_USE_PLANNER
static void CNC_ProcessQueuedStreamingCommands(void)
{
    if (cnc_machine.state == CNC_STATE_HOMING      ||
        cnc_machine.state == CNC_STATE_GOWDELAY    ||
        cnc_machine.state == CNC_STATE_GOWDELAY_RECT ||
        cnc_machine.state == CNC_STATE_ALARM       ||
        cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) return;

    uint8_t budget = 4;
    while (budget-- && !CNC_Queue_IsEmpty() && !CNC_Planner_IsBufferFull()) {
        char cmd[CMD_MAX_LENGTH];
        if (!CNC_Queue_Pop(cmd)) break;
        uint8_t error = CNC_Execute(cmd);
        if (error != 0) CNC_UART_SendError(error);
    }
}
#endif

/*============================================================================
 *                    SYSTICK
 *============================================================================*/

void CNC_SysTick_Callback(void) {
    cnc_systick_ms++;
#if CNC_AUTO_REPORT_ENABLE
    if (cnc_config.auto_report_interval > 0) {
        auto_report_counter++;
        if (auto_report_counter >= cnc_config.auto_report_interval) {
            auto_report_counter  = 0;
            cnc_auto_report_flag = true;
        }
    }
#endif
}

void SysTick_Handler(void) {
    HAL_IncTick();
    CNC_SysTick_Callback();
}

uint32_t CNC_GetTick(void) { return cnc_systick_ms; }

/*============================================================================
 *                    SYSTEM CLOCK (72 MHz)
 *============================================================================*/

void CNC_SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    SysTick_Config(CNC_SYSTEM_CLOCK / 1000);
}

/*============================================================================
 *                    DEFAULT CONFIG
 *============================================================================*/

void CNC_LoadDefaultConfig(void) {
    /* Steps/mm */
    cnc_config.steps_per_mm[0] = DEFAULT_STEPS_PER_MM_X;
    cnc_config.steps_per_mm[1] = DEFAULT_STEPS_PER_MM_Y;
    cnc_config.steps_per_mm[2] = DEFAULT_STEPS_PER_MM_Z;
    cnc_config.steps_per_mm[3] = DEFAULT_STEPS_PER_MM_V;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        if (cnc_config.steps_per_mm[i] < MIN_STEPS_PER_MM)
            cnc_config.steps_per_mm[i] = MIN_STEPS_PER_MM;
        if (cnc_config.steps_per_mm[i] > MAX_STEPS_PER_MM)
            cnc_config.steps_per_mm[i] = MAX_STEPS_PER_MM;
    }
    /* Max feedrate */
    cnc_config.max_feedrate[0] = DEFAULT_MAX_FEEDRATE_X;
    cnc_config.max_feedrate[1] = DEFAULT_MAX_FEEDRATE_Y;
    cnc_config.max_feedrate[2] = DEFAULT_MAX_FEEDRATE_Z;
    cnc_config.max_feedrate[3] = DEFAULT_MAX_FEEDRATE_V;

    /* Max travel */
    cnc_config.max_travel[0] = DEFAULT_MAX_TRAVEL_X;
    cnc_config.max_travel[1] = DEFAULT_MAX_TRAVEL_Y;
    cnc_config.max_travel[2] = DEFAULT_MAX_TRAVEL_Z;
    cnc_config.max_travel[3] = DEFAULT_MAX_TRAVEL_V;

    /* Acceleration */
#if CNC_USE_ACCELERATION
    cnc_config.acceleration[0] = DEFAULT_ACCELERATION_X;
    cnc_config.acceleration[1] = DEFAULT_ACCELERATION_Y;
    cnc_config.acceleration[2] = DEFAULT_ACCELERATION_Z;
    cnc_config.acceleration[3] = DEFAULT_ACCELERATION_V;
#endif

    /* Invert masks */
    cnc_config.step_invert_mask  = DEFAULT_STEP_INVERT_MASK;
    cnc_config.dir_invert_mask   = DEFAULT_DIR_INVERT_MASK;   /* 0b1100: Z,V inverted */
    cnc_config.limit_invert_mask = DEFAULT_LIMIT_INVERT_MASK;
    cnc_config.enable_invert     = DEFAULT_ENABLE_INVERT;

    /* Limits */
    cnc_config.soft_limit_enabled = CNC_SOFT_LIMIT_DEFAULT;
    cnc_config.allow_negative     = CNC_ALLOW_NEGATIVE_COORD;
    cnc_config.hard_limit_enabled = CNC_USE_LIMIT_SWITCH;

    /* Homing */
#if CNC_USE_HOMING
    cnc_config.homing_enabled     = DEFAULT_HOMING_ENABLE;
    cnc_config.homing_dir_mask    = DEFAULT_HOMING_DIR_MASK;
    cnc_config.homing_axis_mask   = DEFAULT_HOMING_AXIS_MASK;
    cnc_config.homing_seek_rate   = DEFAULT_HOMING_SEEK_RATE;
    cnc_config.homing_feed_rate   = DEFAULT_HOMING_FEED_RATE;
    cnc_config.homing_pulloff     = DEFAULT_HOMING_PULLOFF;
    cnc_config.homing_cycle       = DEFAULT_HOMING_CYCLE;
    cnc_config.homing_debounce_ms = DEFAULT_HOMING_DEBOUNCE_MS;
#endif

    /* Backlash */
#if CNC_USE_BACKLASH
    cnc_config.backlash[0]     = DEFAULT_BACKLASH_X;
    cnc_config.backlash[1]     = DEFAULT_BACKLASH_Y;
    cnc_config.backlash[2]     = DEFAULT_BACKLASH_Z;
    cnc_config.backlash[3]     = DEFAULT_BACKLASH_V;
    cnc_config.backlash_enabled = DEFAULT_BACKLASH_ENABLED;
#endif

    /* Auto report */
#if CNC_AUTO_REPORT_ENABLE
    cnc_config.auto_report_interval = CNC_AUTO_REPORT_INTERVAL_MS;
#else
    cnc_config.auto_report_interval = 0;
#endif
}

/*============================================================================
 *                    GPIO INIT
 *============================================================================*/

static void CNC_GPIO_Init(void) {
    /* Enable clocks: GPIOA, GPIOB, AFIO */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    AFIO->MAPR   |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    /*------------------------------------------------------------------------
     * STEP pins - Output 50MHz push-pull
     *   X: PA1   Y: PA0   Z: PA6   V: PB0
     *------------------------------------------------------------------------*/

    /* PA0 (STEP_Y) - CRL bits [3:0] */
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    GPIOA->CRL |=  GPIO_CRL_MODE0;          /* 50MHz output */

    /* PA1 (STEP_X) - CRL bits [7:4] */
    GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
    GPIOA->CRL |=  GPIO_CRL_MODE1;

    /* PA6 (STEP_Z) - CRL bits [27:24] */
    GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
    GPIOA->CRL |=  GPIO_CRL_MODE6;

    /* PB0 (STEP_V) - CRL bits [3:0] */
    GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    GPIOB->CRL |=  GPIO_CRL_MODE0;

    /* All step pins LOW */
    CNC_Step_AllLow();

    /*------------------------------------------------------------------------
     * DIR pins - Output 10MHz push-pull
     *   X: PB5   Y: PB6   Z: PB7   V: PB8
     *------------------------------------------------------------------------*/

    /* PB5 (DIR_X) - CRL bits [23:20] */
    GPIOB->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
    GPIOB->CRL |=  GPIO_CRL_MODE5_0;        /* 10MHz */

    /* PB6 (DIR_Y) - CRL bits [27:24] */
    GPIOB->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
    GPIOB->CRL |=  GPIO_CRL_MODE6_0;

    /* PB7 (DIR_Z) - CRL bits [31:28] */
    GPIOB->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
    GPIOB->CRL |=  GPIO_CRL_MODE7_0;

    /* PB8 (DIR_V) - CRH bits [3:0] */
    GPIOB->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
    GPIOB->CRH |=  GPIO_CRH_MODE8_0;

    /* All DIR pins LOW */
    GPIOB->BRR = (1u << DIR_X_PIN) | (1u << DIR_Y_PIN) |
                 (1u << DIR_Z_PIN) | (1u << DIR_V_PIN);

    /*------------------------------------------------------------------------
     * Enable pins (optional)
     *------------------------------------------------------------------------*/
#if CNC_USE_ENABLE_PIN
    GPIOB->CRH &= ~(GPIO_CRH_CNF10|GPIO_CRH_MODE10);
    GPIOB->CRH |=   GPIO_CRH_MODE10_0;
    GPIOB->CRH &= ~(GPIO_CRH_CNF11|GPIO_CRH_MODE11);
    GPIOB->CRH |=   GPIO_CRH_MODE11_0;
    GPIOB->CRH &= ~(GPIO_CRH_CNF12|GPIO_CRH_MODE12);
    GPIOB->CRH |=   GPIO_CRH_MODE12_0;
    GPIOB->CRH &= ~(GPIO_CRH_CNF9 |GPIO_CRH_MODE9);
    GPIOB->CRH |=   GPIO_CRH_MODE9_0;
    CNC_EnableMotors(false);
#endif
}

/*============================================================================
 *                    STEP CONTROL WITH INVERT
 *============================================================================*/

void CNC_Step_Pulse(uint8_t axis, bool active) {
    bool inverted = (cnc_config.step_invert_mask >> axis) & 1;
    bool output   = active ^ inverted;
    switch (axis) {
        case 0: if (output) STEP_X_HIGH(); else STEP_X_LOW(); break;
        case 1: if (output) STEP_Y_HIGH(); else STEP_Y_LOW(); break;
        case 2: if (output) STEP_Z_HIGH(); else STEP_Z_LOW(); break;
        case 3: if (output) STEP_V_HIGH(); else STEP_V_LOW(); break;
    }
}

void CNC_Step_AllLow(void) {
    for (int i = 0; i < CNC_NUM_AXES; i++) CNC_Step_Pulse(i, false);
}

/*============================================================================
 *                    DIRECTION CONTROL WITH INVERT
 *============================================================================*/

void CNC_SetDirection(uint8_t axis, int8_t dir) {
    if (axis >= CNC_NUM_AXES) return;
    cnc_machine.direction[axis] = dir;

    bool inverted = (cnc_config.dir_invert_mask >> axis) & 1;
    bool output   = (dir > 0) ^ inverted;

    switch (axis) {
        case 0: /* PB5 */
            if (output) DIR_PORT_X->BSRR = (1u << DIR_X_PIN);
            else        DIR_PORT_X->BRR  = (1u << DIR_X_PIN);
            break;
        case 1: /* PB6 */
            if (output) DIR_PORT_Y->BSRR = (1u << DIR_Y_PIN);
            else        DIR_PORT_Y->BRR  = (1u << DIR_Y_PIN);
            break;
        case 2: /* PB7 */
            if (output) DIR_PORT_Z->BSRR = (1u << DIR_Z_PIN);
            else        DIR_PORT_Z->BRR  = (1u << DIR_Z_PIN);
            break;
        case 3: /* PB8 */
            if (output) DIR_PORT_V->BSRR = (1u << DIR_V_PIN);
            else        DIR_PORT_V->BRR  = (1u << DIR_V_PIN);
            break;
    }
}

void CNC_EnableMotors(bool enable) {
#if CNC_USE_ENABLE_PIN
    cnc_machine.motors_enabled = enable;
    uint32_t pins = (1u<<EN_X_PIN)|(1u<<EN_Y_PIN)|(1u<<EN_Z_PIN)|(1u<<EN_V_PIN);
    bool output = enable ^ (bool)cnc_config.enable_invert;
    if (output) EN_PORT->BRR  = pins;
    else        EN_PORT->BSRR = pins;
#else
    cnc_machine.motors_enabled = enable;
#endif
}
/*============================================================================
 *                    TIMER INIT
 *============================================================================*/

static void CNC_Timer_Init(void) {
    RCC->APB1ENR |= CNC_TIMER_RCC;
    CNC_TIMER->CR1  = 0;
    CNC_TIMER->PSC  = 72 - 1;           /* 1 µs tick */
    CNC_TIMER->ARR  = CNC_BASE_TICK_US - 1;
    CNC_TIMER->CR1 |= TIM_CR1_ARPE;
    CNC_TIMER->DIER|= TIM_DIER_UIE;
    CNC_TIMER->EGR |= TIM_EGR_UG;
    CNC_TIMER->SR   = 0;
    NVIC_SetPriority(CNC_TIMER_IRQn, 0);
    NVIC_EnableIRQ(CNC_TIMER_IRQn);
}

/*============================================================================
 *                    STEP TIMING
 *============================================================================*/

static uint32_t CNC_Step_CalcDuty(uint32_t period_us) {
    uint32_t min_period = CNC_STEP_MIN_LOW_US + CNC_STEP_MIN_HIGH_US;
    if (period_us < min_period) period_us = min_period;

    uint32_t low_us  = period_us * (100 - CNC_STEP_DUTY_PERCENT) / 100;
    uint32_t high_us = period_us - low_us;

    if (low_us  < CNC_STEP_MIN_LOW_US)  { low_us  = CNC_STEP_MIN_LOW_US;  high_us = period_us - low_us; }
    if (high_us < CNC_STEP_MIN_HIGH_US) { high_us = CNC_STEP_MIN_HIGH_US; low_us  = period_us - high_us; }
    if (low_us  < CNC_STEP_MIN_LOW_US)    low_us  = CNC_STEP_MIN_LOW_US;
    if (high_us < CNC_STEP_MIN_HIGH_US)   high_us = CNC_STEP_MIN_HIGH_US;

    step_high_ticks = high_us;
    step_low_ticks  = low_us;
    return (step_high_ticks + step_low_ticks);
}

static void CNC_Timer_SetPeriod(uint32_t period_us) {
    if (period_us < MIN_TIMER_PERIOD_US) period_us = MIN_TIMER_PERIOD_US;
    if (period_us > MAX_TIMER_PERIOD_US) period_us = MAX_TIMER_PERIOD_US;
    uint32_t eff = CNC_Step_CalcDuty(period_us);
    CNC_TIMER->ARR = (step_phase == 0) ? step_high_ticks - 1 : step_low_ticks - 1;
#if CNC_USE_ACCELERATION
    current_step_period = eff;
#endif
}

static void CNC_Timer_Start(void) {
    if (step_high_ticks == 0 || step_low_ticks == 0) {
        uint32_t eff = CNC_Step_CalcDuty(CNC_BASE_TICK_US);
#if CNC_USE_ACCELERATION
        current_step_period = eff;
#endif
    }
    step_phase       = 0;
    CNC_TIMER->ARR   = step_high_ticks - 1;
    CNC_TIMER->CNT   = 0;
    CNC_TIMER->SR    = 0;
    CNC_TIMER->CR1  |= TIM_CR1_CEN;
}

static void CNC_Timer_Stop(void) {
    CNC_TIMER->CR1 &= ~TIM_CR1_CEN;
    CNC_TIMER->CNT  = 0;
    step_phase      = 0;
    CNC_Step_AllLow();
    for (int i = 0; i < CNC_NUM_AXES; i++) pending_step[i] = false;
}

/*============================================================================
 *                    ACCELERATION
 *============================================================================*/

#if CNC_USE_ACCELERATION

static inline float CNC_SafeStepsPerMM(uint8_t axis) {
    if (axis >= CNC_NUM_AXES) return 1.0f;
    float spm = cnc_config.steps_per_mm[axis];
    if (spm < MIN_STEPS_PER_MM) spm = MIN_STEPS_PER_MM;
    if (spm > MAX_STEPS_PER_MM) spm = MAX_STEPS_PER_MM;
    return spm;
}

void CNC_Accel_Update(void)
{
    AccelState_t *a = &cnc_machine.accel;
    if (!cnc_machine.is_moving) return;
    if (cnc_machine.state != CNC_STATE_RUN &&
        cnc_machine.state != CNC_STATE_GOTO) return;

    uint32_t now        = CNC_GetTick();
    uint32_t elapsed_ms = now - a->last_update_tick;
    if (elapsed_ms < CNC_ACCEL_UPDATE_MS) return;
    a->last_update_tick = now;

    float dt_s = (float)elapsed_ms * 0.001f;
    if (dt_s > 0.1f) dt_s = 0.1f;

    __disable_irq();
    int64_t total_snap     = bresenham_total;
    int64_t remaining_snap = step_events_remaining;
    __enable_irq();

    if (total_snap <= 0) return;
    if (remaining_snap < 0)          remaining_snap = 0;
    if (remaining_snap > total_snap) remaining_snap = total_snap;
    if (total_snap > (int64_t)0xFFFFFFFFLL) {
        a->phase = ACCEL_PHASE_CRUISE; a->current_speed = a->target_speed;
        goto compute_timing;
    }

    uint32_t total_steps     = (uint32_t)total_snap;
    uint32_t steps_remaining = (uint32_t)remaining_snap;
    uint32_t steps_done      = total_steps - steps_remaining;

    if (total_steps == 0) return;

    float mm_per_step = a->mm_per_step;
    if (mm_per_step <= 0.0f) {
        float dist = cnc_machine.move_distance;
        if (dist < 0.001f) return;
        mm_per_step    = dist / (float)total_steps;
        a->mm_per_step = mm_per_step;
    }

    float dist_remaining = (float)steps_remaining * mm_per_step;

    if (steps_remaining == 0) {
        a->phase = ACCEL_PHASE_DECEL; a->current_speed = a->exit_speed;
        goto compute_timing;
    } else if (steps_done >= a->decel_start) {
        a->phase = ACCEL_PHASE_DECEL;
    } else if (steps_done >= a->accel_steps) {
        a->phase = ACCEL_PHASE_CRUISE;
    }

    switch (a->phase) {
        case ACCEL_PHASE_ACCEL: {
            float ns = a->current_speed + a->accel_rate * dt_s;
            if (ns > a->target_speed) ns = a->target_speed;
            if (ns < a->entry_speed)  ns = a->entry_speed;
            a->current_speed = ns; break;
        }
        case ACCEL_PHASE_CRUISE:
            a->current_speed = a->target_speed; break;
        case ACCEL_PHASE_DECEL: {
            float ns = a->current_speed - a->accel_rate * dt_s;
            if (ns < a->exit_speed)   ns = a->exit_speed;
            if (ns > a->target_speed) ns = a->target_speed;
            a->current_speed = ns; break;
        }
        default: break;
    }

    /* Safety envelope */
    {
        float safe_sqr = a->exit_speed * a->exit_speed
                       + 2.0f * a->accel_rate * dist_remaining;
        if (safe_sqr < 0.0f) safe_sqr = 0.0f;
        float max_safe = sqrtf(safe_sqr);
        if (max_safe < a->exit_speed)   max_safe = a->exit_speed;
        if (max_safe > a->target_speed) max_safe = a->target_speed;
        if (a->current_speed > max_safe) a->current_speed = max_safe;
    }

compute_timing:
    {
        float spm   = a->spm_dominant;
        if (spm < 0.01f) spm = 1.0f;
        float speed = a->current_speed;
        if (speed < 0.0f) speed = 0.0f;
        float min_speed = (float)CNC_MIN_STEP_FREQ * a->inv_spm_dominant;
        if (min_speed < 0.001f) min_speed = 0.001f;
        if (speed < min_speed) speed = min_speed;

        float freq = speed * spm;
        if (freq < (float)CNC_MIN_STEP_FREQ) freq = (float)CNC_MIN_STEP_FREQ;
        if (freq > 500000.0f)                freq = 500000.0f;

        uint32_t new_period = (uint32_t)(1000000.0f / freq);
        if (new_period < MIN_TIMER_PERIOD_US) new_period = MIN_TIMER_PERIOD_US;
        if (new_period > MAX_TIMER_PERIOD_US) new_period = MAX_TIMER_PERIOD_US;

        uint32_t min_p = CNC_STEP_MIN_LOW_US + CNC_STEP_MIN_HIGH_US;
        if (new_period < min_p) new_period = min_p;

        uint32_t low_us  = new_period * (100 - CNC_STEP_DUTY_PERCENT) / 100;
        uint32_t high_us = new_period - low_us;
        if (low_us  < CNC_STEP_MIN_LOW_US)  { low_us  = CNC_STEP_MIN_LOW_US;  high_us = new_period - low_us; }
        if (high_us < CNC_STEP_MIN_HIGH_US) { high_us = CNC_STEP_MIN_HIGH_US; low_us  = new_period - high_us; }
        if (low_us  < CNC_STEP_MIN_LOW_US)    low_us  = CNC_STEP_MIN_LOW_US;
        if (high_us < CNC_STEP_MIN_HIGH_US)   high_us = CNC_STEP_MIN_HIGH_US;

        __disable_irq();
        a->precomp_high_ticks = high_us;
        a->precomp_low_ticks  = low_us;
        a->precomp_period     = high_us + low_us;
        a->needs_recalc       = true;
        __enable_irq();
    }
}

void CNC_Accel_TimerTick(void)
{
    AccelState_t *a = &cnc_machine.accel;
    if (!a->needs_recalc) return;
    a->needs_recalc = false;

    uint32_t nh = a->precomp_high_ticks;
    uint32_t nl = a->precomp_low_ticks;
    uint32_t np = a->precomp_period;

    if (nh < CNC_STEP_MIN_HIGH_US) nh = CNC_STEP_MIN_HIGH_US;
    if (nl < CNC_STEP_MIN_LOW_US)  nl = CNC_STEP_MIN_LOW_US;
    if (np < (CNC_STEP_MIN_HIGH_US + CNC_STEP_MIN_LOW_US))
        np = CNC_STEP_MIN_HIGH_US + CNC_STEP_MIN_LOW_US;

    step_high_ticks     = nh;
    step_low_ticks      = nl;
    current_step_period = np;
}

void CNC_Accel_Plan(float distance, float feedrate, float accel)
{
    uint8_t dom = cnc_machine.dominant_axis;
    float   spm = CNC_SafeStepsPerMM(dom);
    if (spm < 0.01f) spm = 1.0f;
    float min_speed = (float)CNC_MIN_STEP_FREQ / spm;
    CNC_Accel_PlanWithEntryExit(distance, feedrate, accel, min_speed, min_speed);
}

void CNC_Accel_PlanWithEntryExit(float distance, float feedrate, float accel,
                                  float entry_speed, float exit_speed)
{
    AccelState_t *a   = &cnc_machine.accel;
    uint8_t       dom = cnc_machine.dominant_axis;
    float         spm = CNC_SafeStepsPerMM(dom);
    if (spm < 0.01f) spm = 1.0f;

    a->mm_per_step      = 0.0f;
    a->last_update_tick = CNC_GetTick();
    a->needs_recalc     = false;
    a->spm_dominant     = spm;
    a->inv_spm_dominant = 1.0f / spm;

    float cruise_speed = feedrate / 60.0f;
    float min_speed    = (float)CNC_MIN_STEP_FREQ / spm;
    if (min_speed < 0.001f) min_speed = 0.001f;

    if (entry_speed  < min_speed) entry_speed  = min_speed;
    if (exit_speed   < min_speed) exit_speed   = min_speed;
    if (cruise_speed < min_speed) cruise_speed = min_speed;
    if (accel < 0.1f) accel = 0.1f;

    if (distance < 0.001f) {
        a->phase = ACCEL_PHASE_CRUISE; a->current_speed = cruise_speed;
        a->target_speed = cruise_speed; a->entry_speed = cruise_speed;
        a->exit_speed = cruise_speed; a->accel_rate = accel;
        a->accel_steps = a->cruise_steps = a->decel_steps = a->decel_start = 0;
        return;
    }

    a->target_speed = cruise_speed; a->entry_speed = entry_speed;
    a->exit_speed   = exit_speed;   a->accel_rate  = accel;
    a->current_speed = entry_speed;

    float accel_dist = (cruise_speed*cruise_speed - entry_speed*entry_speed) / (2.0f*accel);
    float decel_dist = (cruise_speed*cruise_speed - exit_speed*exit_speed)   / (2.0f*accel);
    if (accel_dist < 0.0f) accel_dist = 0.0f;
    if (decel_dist < 0.0f) decel_dist = 0.0f;

    if (accel_dist + decel_dist > distance) {
        float pk_sqr = accel*distance + 0.5f*(entry_speed*entry_speed + exit_speed*exit_speed);
        if (pk_sqr < 0.0f) pk_sqr = 0.0f;
        float peak = sqrtf(pk_sqr);
        if (peak < min_speed) peak = min_speed;
        accel_dist = (peak*peak - entry_speed*entry_speed) / (2.0f*accel);
        decel_dist = distance - accel_dist;
        if (accel_dist < 0.0f) accel_dist = 0.0f;
        if (decel_dist < 0.0f) decel_dist = 0.0f;
        a->target_speed = peak;
    }

    float cruise_dist = distance - accel_dist - decel_dist;
    if (cruise_dist < 0.0f) cruise_dist = 0.0f;

    __disable_irq();
    int64_t bt = bresenham_total;
    __enable_irq();

    if (bt <= 0 || bt > (int64_t)0xFFFFFFFFLL) {
        a->phase = ACCEL_PHASE_CRUISE; a->current_speed = cruise_speed;
        a->accel_steps = a->cruise_steps = a->decel_steps = a->decel_start = 0;
        return;
    }

    uint32_t total_steps = (uint32_t)bt;
    a->mm_per_step  = distance / (float)total_steps;
    a->accel_steps  = (uint32_t)((accel_dist  / distance) * (float)total_steps);
    a->cruise_steps = (uint32_t)((cruise_dist / distance) * (float)total_steps);
    if (a->accel_steps > total_steps) { a->accel_steps = total_steps; a->cruise_steps = 0; }
    if (a->accel_steps + a->cruise_steps > total_steps)
        a->cruise_steps = total_steps - a->accel_steps;
    a->decel_steps = total_steps - a->accel_steps - a->cruise_steps;
    a->decel_start = a->accel_steps + a->cruise_steps;
    if (a->decel_start >= total_steps)
        a->decel_start = (total_steps > 1) ? (total_steps - 1) : 0;

    if (a->accel_steps == 0) {
        a->phase         = (a->cruise_steps == 0) ? ACCEL_PHASE_DECEL : ACCEL_PHASE_CRUISE;
        a->current_speed = a->target_speed;
    } else {
        a->phase = ACCEL_PHASE_ACCEL; a->current_speed = entry_speed;
    }

    /* First timing */
    float freq = entry_speed * spm;
    if (freq < (float)CNC_MIN_STEP_FREQ) freq = (float)CNC_MIN_STEP_FREQ;
    if (freq > 500000.0f)                freq = 500000.0f;
    uint32_t period = (uint32_t)(1000000.0f / freq);
    if (period < MIN_TIMER_PERIOD_US) period = MIN_TIMER_PERIOD_US;
    if (period > MAX_TIMER_PERIOD_US) period = MAX_TIMER_PERIOD_US;
    current_step_period    = CNC_Step_CalcDuty(period);
    a->precomp_high_ticks  = step_high_ticks;
    a->precomp_low_ticks   = step_low_ticks;
    a->precomp_period      = current_step_period;
    a->needs_recalc        = false;
}

uint32_t CNC_Accel_GetCurrentFreq(void)
{
    float spm   = cnc_machine.accel.spm_dominant;
    if (spm < 0.01f) spm = 1.0f;
    float freq  = cnc_machine.accel.current_speed * spm;
    if (freq < (float)CNC_MIN_STEP_FREQ) freq = (float)CNC_MIN_STEP_FREQ;
    if (freq > 500000.0f)                freq = 500000.0f;
    return (uint32_t)freq;
}

float CNC_Accel_GetCurrentSpeed(void)
{
    float s = cnc_machine.accel.current_speed;
    return (s < 0.0f) ? 0.0f : s;
}

#else /* !CNC_USE_ACCELERATION */

static inline float CNC_SafeStepsPerMM(uint8_t axis) {
    if (axis >= CNC_NUM_AXES) return 1.0f;
    float spm = cnc_config.steps_per_mm[axis];
    if (spm < MIN_STEPS_PER_MM) spm = MIN_STEPS_PER_MM;
    if (spm > MAX_STEPS_PER_MM) spm = MAX_STEPS_PER_MM;
    return spm;
}

#endif /* CNC_USE_ACCELERATION */

/*============================================================================
 *                    TIMER ISR
 *============================================================================*/

void CNC_TIMER_IRQHandler(void) {
    if (!(CNC_TIMER->SR & TIM_SR_UIF)) return;
    CNC_TIMER->SR = ~TIM_SR_UIF;

    if (!cnc_machine.is_moving) return;

    /*--- Phase 1: LOW - end step pulse ---*/
    if (step_phase == 1) {
        bool had = false;
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            if (pending_step[i]) {
                CNC_Step_Pulse(i, false);
                pending_step[i] = false;
                had = true;
                cnc_machine.position[i] +=
                    (float)cnc_machine.direction[i] / cnc_config.steps_per_mm[i];
            }
        }
        if (had) {
            step_events_remaining--;
#if CNC_USE_ACCELERATION
            if (cnc_machine.state == CNC_STATE_RUN ||
                cnc_machine.state == CNC_STATE_GOTO)
                CNC_Accel_TimerTick();
#endif
#if CNC_USE_HOMING
            if (cnc_machine.state == CNC_STATE_HOMING) homing_step_count++;
#endif
        }
        step_phase = 0;
        if (step_high_ticks == 0)
            CNC_Step_CalcDuty(current_step_period > 0 ? current_step_period : CNC_BASE_TICK_US);
        CNC_TIMER->ARR = step_high_ticks - 1;
        return;
    }

    /*--- Limit check ---*/
#if CNC_USE_LIMIT_SWITCH
    if (cnc_config.hard_limit_enabled &&
        (cnc_machine.state == CNC_STATE_RUN      ||
         cnc_machine.state == CNC_STATE_GOTO      ||
         cnc_machine.state == CNC_STATE_GOWDELAY ||
         cnc_machine.state == CNC_STATE_GOWDELAY_RECT))
    {
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            if (CNC_Limit_IsTriggered(i) && cnc_machine.step_count[i] > 0) {
                cnc_machine.limit_triggered[i] = true;
                CNC_Timer_Stop();
                cnc_machine.is_moving = false;
                cnc_machine.state = CNC_STATE_LIMIT_TRIGGERED;
                return;
            }
        }
    }
#endif

    /*--- Homing limit ---*/
#if CNC_USE_HOMING
    if (cnc_machine.state == CNC_STATE_HOMING &&
        (cnc_machine.homing_phase == HOMING_SEEK ||
         cnc_machine.homing_phase == HOMING_FEED))
    {
        if (CNC_Limit_IsTriggered(cnc_machine.homing_axis)) {
            homing_limit_hit  = true;
            CNC_Timer_Stop();
            cnc_machine.is_moving = false;
            return;
        }
    }
#endif

    /*--- Check completion ---*/
    if (step_events_remaining <= 0) {
        CNC_Timer_Stop();
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            cnc_machine.position[i] = cnc_machine.target[i];
            cnc_machine.step_count[i] = 0;
        }
        cnc_machine.is_moving = false;
        if (cnc_machine.state == CNC_STATE_RUN) {
            cnc_machine.state    = CNC_STATE_IDLE;
            cnc_send_ok_flag     = true;
        } else if (cnc_machine.state == CNC_STATE_GOTO) {
            cnc_machine.state = CNC_STATE_IDLE;
        }
        return;
    }

    /*--- Bresenham ---*/
    bool any_step = false;
    for (int axis = 0; axis < CNC_NUM_AXES; axis++) {
        if (bresenham_steps[axis] == 0) continue;
        bresenham_counter[axis] += bresenham_steps[axis];
        if (bresenham_counter[axis] >= bresenham_total) {
            bresenham_counter[axis] -= bresenham_total;
            CNC_Step_Pulse(axis, true);
            pending_step[axis] = true;
            any_step = true;
        }
    }

    if (any_step) {
        step_phase = 1;
        if (step_low_ticks == 0)
            CNC_Step_CalcDuty(current_step_period > 0 ? current_step_period : CNC_BASE_TICK_US);
        CNC_TIMER->ARR = step_low_ticks - 1;
    } else {
        if (step_high_ticks > 0) CNC_TIMER->ARR = step_high_ticks - 1;
    }
}

/*============================================================================
 *                    LIMIT SWITCH
 *============================================================================*/

#if CNC_USE_LIMIT_SWITCH

void CNC_Limit_Init(void) {
    /* PA3=X, PA4=Y, PA5=Z as input pull-up */
    GPIOA->CRL &= ~(GPIO_CRL_CNF3|GPIO_CRL_MODE3|
                    GPIO_CRL_CNF4|GPIO_CRL_MODE4|
                    GPIO_CRL_CNF5|GPIO_CRL_MODE5);
    GPIOA->CRL |= (GPIO_CRL_CNF3_1 | GPIO_CRL_CNF4_1 | GPIO_CRL_CNF5_1);
    GPIOA->BSRR = (1u<<LIMIT_X_PIN)|(1u<<LIMIT_Y_PIN)|(1u<<LIMIT_Z_PIN);

#if LIMIT_V_ENABLED
    GPIOA->CRL &= ~(GPIO_CRL_CNF6|GPIO_CRL_MODE6);
    GPIOA->CRL |=  GPIO_CRL_CNF6_1;
    GPIOA->BSRR = (1u<<LIMIT_V_PIN);
#endif
    for (int i = 0; i < CNC_NUM_AXES; i++) cnc_machine.limit_triggered[i] = false;
}

bool CNC_Limit_IsTriggered(uint8_t axis) {
    uint16_t pin = 0;
    GPIO_TypeDef *port = LIMIT_PORT;
    switch (axis) {
        case 0: pin = (1u<<LIMIT_X_PIN); break;
        case 1: pin = (1u<<LIMIT_Y_PIN); break;
        case 2: pin = (1u<<LIMIT_Z_PIN); break;
#if LIMIT_V_ENABLED
        case 3: pin = (1u<<LIMIT_V_PIN); break;
#endif
        default: return false;
    }
    bool state    = (port->IDR & pin) != 0;
    bool inverted = (cnc_config.limit_invert_mask >> axis) & 1;
    return state ^ (!inverted);
}

bool CNC_Limit_CheckAll(void)
{
    if (!cnc_config.hard_limit_enabled) return false;
    bool any = false;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        if (CNC_Limit_IsTriggered(i)) { cnc_machine.limit_triggered[i] = true; any = true; }
    }
    if (any && cnc_machine.is_moving &&
        (cnc_machine.state == CNC_STATE_RUN  ||
         cnc_machine.state == CNC_STATE_GOTO ||
         cnc_machine.state == CNC_STATE_GOWDELAY ||
         cnc_machine.state == CNC_STATE_GOWDELAY_RECT)) {
        CNC_Stop();
        cnc_machine.state = CNC_STATE_LIMIT_TRIGGERED;
        CNC_UART_PutString("[LIMIT:");
        for (int i = 0; i < CNC_NUM_AXES; i++)
            if (cnc_machine.limit_triggered[i]) { CNC_UART_PutChar(' '); CNC_UART_PutChar(CNC_AxisName(i)); }
        CNC_UART_PutString("]\r\n");
    }
    return any;
}

void CNC_Limit_ClearAlarm(void) {
    for (int i = 0; i < CNC_NUM_AXES; i++) cnc_machine.limit_triggered[i] = false;
    if (cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED ||
        cnc_machine.state == CNC_STATE_ALARM) {
        cnc_machine.state = CNC_STATE_IDLE;
        CNC_UART_PutString("[ALARM CLEARED]\r\n");
    }
}

#endif /* CNC_USE_LIMIT_SWITCH */

/*============================================================================
 *                    RELAY
 *============================================================================*/

#if CNC_USE_RELAY

void CNC_Relay_Init(void) {
    GPIOB->CRH &= ~(GPIO_CRH_CNF13|GPIO_CRH_MODE13|
                    GPIO_CRH_CNF14|GPIO_CRH_MODE14|
                    GPIO_CRH_CNF15|GPIO_CRH_MODE15);
    GPIOB->CRH |= GPIO_CRH_MODE13_0|GPIO_CRH_MODE14_0|GPIO_CRH_MODE15_0;
    CNC_Spindle_Off(); CNC_Coolant_Off();
}

#define _RELAY_ON(PORT, PIN)  do { \
    if (RELAY_ACTIVE_HIGH) (PORT)->BSRR = (1u<<(PIN)); \
    else                   (PORT)->BRR  = (1u<<(PIN)); } while(0)

#define _RELAY_OFF(PORT, PIN) do { \
    if (RELAY_ACTIVE_HIGH) (PORT)->BRR  = (1u<<(PIN)); \
    else                   (PORT)->BSRR = (1u<<(PIN)); } while(0)

void CNC_Spindle_On(void)       { _RELAY_ON (RELAY_SPINDLE_PORT,      RELAY_SPINDLE_PIN);      cnc_machine.spindle_on      = true;  }
void CNC_Spindle_Off(void)      { _RELAY_OFF(RELAY_SPINDLE_PORT,      RELAY_SPINDLE_PIN);      cnc_machine.spindle_on      = false; }
void CNC_Coolant_Mist_On(void)  { _RELAY_ON (RELAY_COOLANT_MIST_PORT, RELAY_COOLANT_MIST_PIN); cnc_machine.coolant_mist_on = true;  }
void CNC_Coolant_Mist_Off(void) { _RELAY_OFF(RELAY_COOLANT_MIST_PORT, RELAY_COOLANT_MIST_PIN); cnc_machine.coolant_mist_on = false; }
void CNC_Coolant_Flood_On(void) { _RELAY_ON (RELAY_COOLANT_FLOOD_PORT,RELAY_COOLANT_FLOOD_PIN);cnc_machine.coolant_flood_on= true;  }
void CNC_Coolant_Flood_Off(void){ _RELAY_OFF(RELAY_COOLANT_FLOOD_PORT,RELAY_COOLANT_FLOOD_PIN);cnc_machine.coolant_flood_on= false; }
void CNC_Coolant_Off(void)      { CNC_Coolant_Mist_Off(); CNC_Coolant_Flood_Off(); }

#endif /* CNC_USE_RELAY */

/*============================================================================
 *                    HOMING
 *============================================================================*/

#if CNC_USE_HOMING

static void CNC_Homing_MoveAxis(uint8_t axis, float distance, float feedrate) {
    if (axis >= CNC_NUM_AXES) return;
    float target[CNC_NUM_AXES];
    CNC_GetPosition(target);
    target[axis] += distance;
    cnc_machine.target[axis] = target[axis];

    int64_t hsteps = (int64_t)fabsf(distance * cnc_config.steps_per_mm[axis]);
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        bresenham_steps[i]       = (i == axis) ? hsteps : 0;
        cnc_machine.step_count[i]= (i == axis) ? (int32_t)hsteps : 0;
    }
    if (hsteps == 0) return;

    __disable_irq();
    bresenham_total       = hsteps;
    step_events_remaining = hsteps;
    __enable_irq();

    for (int i = 0; i < CNC_NUM_AXES; i++)
        bresenham_counter[i] = bresenham_total / 2;

    CNC_SetDirection(axis, distance > 0 ? 1 : -1);

    uint32_t step_freq = (uint32_t)((feedrate / 60.0f) * cnc_config.steps_per_mm[axis]);
    if (step_freq < 10)    step_freq = 10;
    if (step_freq > 50000) step_freq = 50000;
    CNC_Timer_SetPeriod(1000000UL / step_freq);

    homing_step_count = 0;
    homing_limit_hit  = false;
    cnc_machine.is_moving = true;
    CNC_Timer_Start();
}

static int8_t CNC_Homing_GetNextAxis(void) {
    uint8_t pending = cnc_machine.homing_axes_pending;
    if (!pending) return -1;
    switch (cnc_config.homing_cycle) {
        case 1:
            if (pending & 0x04) return 2;
            if (pending & 0x01) return 0;
            if (pending & 0x02) return 1;
            break;
        case 2:
            if (pending & 0x04) return 2;
            if (pending & 0x01) return 0;
            if (pending & 0x02) return 1;
            break;
        default:
            for (int i = 0; i < CNC_NUM_AXES; i++)
                if (pending & (1<<i)) return i;
            break;
    }
    return -1;
}

void CNC_HomeAxis(uint8_t axis) {
    if (axis >= CNC_NUM_AXES) return;
    if (!cnc_config.homing_enabled) { CNC_UART_PutString("[HOMING DISABLED]\r\n"); return; }
    if (!(cnc_config.homing_axis_mask & (1<<axis))) {
        CNC_UART_PutString("[AXIS "); CNC_UART_PutChar(CNC_AxisName(axis));
        CNC_UART_PutString(" NOT IN MASK]\r\n"); return;
    }
#if CNC_USE_LIMIT_SWITCH
    cnc_machine.state                = CNC_STATE_HOMING;
    cnc_machine.homing_axis          = axis;
    cnc_machine.homing_phase         = HOMING_SEEK;
    cnc_machine.homing_axes_pending  = (1<<axis);
    cnc_machine.homing_cycle_step    = 0;
    cnc_machine.homed[axis]          = false;
    CNC_EnableMotors(true);
    float dir = ((cnc_config.homing_dir_mask >> axis) & 1) ? -1.0f : 1.0f;
    CNC_UART_PutString("[HOMING "); CNC_UART_PutChar(CNC_AxisName(axis));
    CNC_UART_PutString(dir<0?" DIR:-":" DIR:+"); CNC_UART_PutString(" SEEK]\r\n");
    CNC_Homing_MoveAxis(axis, dir * cnc_config.max_travel[axis] * 1.5f, cnc_config.homing_seek_rate);
#else
    cnc_machine.position[axis] = 0; cnc_machine.homed[axis] = false;
    CNC_UART_PutString("["); CNC_UART_PutChar(CNC_AxisName(axis));
    CNC_UART_PutString(" RESET - NO SWITCH]\r\n");
#endif
}

void CNC_HomeAll(void) {
    if (!cnc_config.homing_enabled) {
        for (int i=0;i<CNC_NUM_AXES;i++){cnc_machine.position[i]=0;cnc_machine.homed[i]=false;}
        CNC_UART_PutString("[POSITION RESET - HOMING DISABLED]\r\n"); return;
    }
    if (!cnc_config.homing_axis_mask) { CNC_UART_PutString("[NO AXES IN MASK]\r\n"); return; }
#if CNC_USE_LIMIT_SWITCH
    cnc_machine.state               = CNC_STATE_HOMING;
    cnc_machine.homing_axes_pending = cnc_config.homing_axis_mask;
    cnc_machine.homing_cycle_step   = 0;
    for (int i=0;i<CNC_NUM_AXES;i++)
        if (cnc_config.homing_axis_mask & (1<<i)) cnc_machine.homed[i] = false;
    CNC_EnableMotors(true);

    int8_t next = CNC_Homing_GetNextAxis();
    if (next >= 0) {
        cnc_machine.homing_axis  = next;
        cnc_machine.homing_phase = HOMING_SEEK;
        float dir = ((cnc_config.homing_dir_mask >> next) & 1) ? -1.0f : 1.0f;
        CNC_Homing_MoveAxis(next, dir * cnc_config.max_travel[next] * 1.5f, cnc_config.homing_seek_rate);
    }
#else
    for (int i=0;i<CNC_NUM_AXES;i++)
        if (cnc_config.homing_axis_mask&(1<<i)){cnc_machine.position[i]=0;cnc_machine.homed[i]=true;}
    CNC_UART_PutString("[HOMED - NO SWITCHES]\r\n");
#endif
}

void CNC_Homing_Process(void) {
    if (cnc_machine.state != CNC_STATE_HOMING) return;
    if (cnc_machine.is_moving) return;

    uint8_t axis = cnc_machine.homing_axis;
    float dir = ((cnc_config.homing_dir_mask >> axis) & 1) ? -1.0f : 1.0f;

    switch (cnc_machine.homing_phase) {
        case HOMING_SEEK:
            if (homing_limit_hit) {
                cnc_machine.homing_phase = HOMING_PULLOFF1;
                CNC_Homing_MoveAxis(axis, -dir * cnc_config.homing_pulloff * 3, cnc_config.homing_seek_rate);
            } else {
                cnc_machine.state = CNC_STATE_ALARM;
                CNC_UART_PutString("[HOMING FAIL - NO SWITCH]\r\n");
            }
            break;

        case HOMING_PULLOFF1:
            cnc_machine.homing_phase = HOMING_FEED;
            CNC_Homing_MoveAxis(axis, dir * cnc_config.homing_pulloff * 5, cnc_config.homing_feed_rate);
            break;

        case HOMING_FEED:
            if (homing_limit_hit) {
                cnc_machine.homing_phase = HOMING_PULLOFF2;
                CNC_Homing_MoveAxis(axis, -dir * cnc_config.homing_pulloff, cnc_config.homing_feed_rate);
            } else {
                cnc_machine.state = CNC_STATE_ALARM;
                CNC_UART_PutString("[HOMING FAIL - FEED NO HIT]\r\n");
            }
            break;

        case HOMING_PULLOFF2:
            cnc_machine.position[axis] = 0;
            cnc_machine.homed[axis]    = true;
            cnc_machine.homing_axes_pending &= ~(1<<axis);
            CNC_UART_PutString("["); CNC_UART_PutChar(CNC_AxisName(axis));
            CNC_UART_PutString(" HOMED]\r\n");
            {
                int8_t next = CNC_Homing_GetNextAxis();
                if (next >= 0) {
                    cnc_machine.homing_axis  = next;
                    cnc_machine.homing_phase = HOMING_SEEK;
                    cnc_machine.homing_cycle_step++;
                    float nd = ((cnc_config.homing_dir_mask>>next)&1)?-1.0f:1.0f;
                    CNC_Homing_MoveAxis(next, nd*cnc_config.max_travel[next]*1.5f, cnc_config.homing_seek_rate);
                } else {
                    cnc_machine.homing_phase = HOMING_COMPLETE;
                    cnc_machine.state        = CNC_STATE_IDLE;
                    CNC_UART_PutString("[HOMING COMPLETE]\r\n");
                }
            }
            break;

        default:
            cnc_machine.state = CNC_STATE_IDLE;
            break;
    }
}

bool CNC_Homing_IsActive(void) { return cnc_machine.state == CNC_STATE_HOMING; }

#endif /* CNC_USE_HOMING */

/*============================================================================
 *                    DMA UART
 *============================================================================*/

#if CNC_USE_DMA_UART

void CNC_UART_DMA_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    uart_dma_rx.write_pos = uart_dma_rx.read_pos = 0;

    CNC_UART_DMA_CHANNEL->CCR   = 0;
    DMA1->IFCR = CNC_UART_DMA_FLAG_TC | CNC_UART_DMA_FLAG_HT;
    CNC_UART_DMA_CHANNEL->CPAR  = (uint32_t)&(CNC_UART->DR);
    CNC_UART_DMA_CHANNEL->CMAR  = (uint32_t)uart_dma_rx.buffer;
    CNC_UART_DMA_CHANNEL->CNDTR = UART_DMA_BUFFER_SIZE;
    CNC_UART_DMA_CHANNEL->CCR   = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_0;
    CNC_UART->CR3 |= USART_CR3_DMAR;
    CNC_UART_DMA_CHANNEL->CCR  |= DMA_CCR_EN;
}

uint16_t CNC_UART_DMA_Available(void) {
    uint16_t dma_counter     = CNC_UART_DMA_CHANNEL->CNDTR;
    uart_dma_rx.write_pos    = UART_DMA_BUFFER_SIZE - dma_counter;
    if (uart_dma_rx.write_pos >= uart_dma_rx.read_pos)
        return uart_dma_rx.write_pos - uart_dma_rx.read_pos;
    return UART_DMA_BUFFER_SIZE - uart_dma_rx.read_pos + uart_dma_rx.write_pos;
}

uint8_t CNC_UART_DMA_Read(void) {
    if (!CNC_UART_DMA_Available()) return 0;
    uint8_t data = uart_dma_rx.buffer[uart_dma_rx.read_pos];
    if (++uart_dma_rx.read_pos >= UART_DMA_BUFFER_SIZE) uart_dma_rx.read_pos = 0;
    return data;
}

void CNC_UART_DMA_Flush(void) {
    uint16_t c = CNC_UART_DMA_CHANNEL->CNDTR;
    uart_dma_rx.write_pos = uart_dma_rx.read_pos = UART_DMA_BUFFER_SIZE - c;
}

#endif

/*============================================================================
 *                    COMMAND QUEUE
 *============================================================================*/

#if CNC_USE_COMMAND_QUEUE
void CNC_Queue_Init(void)  { cmd_queue.head = cmd_queue.tail = cmd_queue.count = 0; }
bool CNC_Queue_IsFull(void)  { return cmd_queue.count >= CMD_QUEUE_SIZE; }
bool CNC_Queue_IsEmpty(void) { return cmd_queue.count == 0; }
uint8_t CNC_Queue_Count(void)     { return cmd_queue.count; }
uint8_t CNC_Queue_Available(void) { return CMD_QUEUE_SIZE - cmd_queue.count; }
void CNC_Queue_Clear(void)  { cmd_queue.head = cmd_queue.tail = cmd_queue.count = 0; }

bool CNC_Queue_Push(const char *cmd) {
    if (CNC_Queue_IsFull()) return false;
    strncpy(cmd_queue.commands[cmd_queue.head], cmd, CMD_MAX_LENGTH-1);
    cmd_queue.commands[cmd_queue.head][CMD_MAX_LENGTH-1] = '\0';
    cmd_queue.head = (cmd_queue.head + 1) % CMD_QUEUE_SIZE;
    cmd_queue.count++; return true;
}

bool CNC_Queue_Pop(char *cmd) {
    if (CNC_Queue_IsEmpty()) return false;
    strncpy(cmd, cmd_queue.commands[cmd_queue.tail], CMD_MAX_LENGTH);
    cmd_queue.tail = (cmd_queue.tail + 1) % CMD_QUEUE_SIZE;
    cmd_queue.count--; return true;
}
#endif

/*============================================================================
 *                    MOTION PLANNER
 *============================================================================*/

#if CNC_USE_PLANNER

void CNC_Planner_Init(void) {
    planner.head = planner.tail = planner.count = 0;
    for (int i=0;i<CNC_NUM_AXES;i++) planner.previous_unit_vec[i] = 0;
    planner.previous_nominal_speed = 0;
}

bool CNC_Planner_AddBlock(float *target, float feedrate, bool rapid) {
    if (CNC_Planner_IsBufferFull()) return false;
    PlannerBlock_t *block = &planner.blocks[planner.head];

    float current_pos[CNC_NUM_AXES];
    if (planner.count > 0) {
        uint8_t li = (planner.head==0)?(PLANNER_BUFFER_SIZE-1):(planner.head-1);
        for (int i=0;i<CNC_NUM_AXES;i++) current_pos[i] = planner.blocks[li].target[i];
    } else {
        CNC_GetPosition(current_pos);
    }

    for (int i=0;i<CNC_NUM_AXES;i++) block->target[i] = target[i];
    block->feedrate = feedrate; block->rapid = rapid;

    float delta[CNC_NUM_AXES];
    block->distance = 0;
    for (int i=0;i<CNC_NUM_AXES;i++) {
        delta[i] = target[i] - current_pos[i];
        block->distance += delta[i]*delta[i];
    }
    block->distance = sqrtf(block->distance);
    if (block->distance < 0.001f) return false;

    for (int i=0;i<CNC_NUM_AXES;i++) block->unit_vec[i] = delta[i] / block->distance;

    block->acceleration = CNC_ComputeVectorMaxAccelMMs2(block->unit_vec);
    if (block->acceleration < 0.1f) block->acceleration = 0.1f;

    float vector_max_speed = CNC_ComputeVectorMaxSpeedMMs(block->unit_vec);
    if (rapid) {
        block->nominal_speed = vector_max_speed;
    } else {
        float rs = feedrate / 60.0f;
        if (rs < PLANNER_MIN_SPEED) rs = PLANNER_MIN_SPEED;
        block->nominal_speed = CNC_min(rs, vector_max_speed);
    }
    if (block->nominal_speed < PLANNER_MIN_SPEED) block->nominal_speed = PLANNER_MIN_SPEED;
    block->nominal_speed_sqr = block->nominal_speed * block->nominal_speed;
    block->max_entry_speed_sqr = block->nominal_speed_sqr;
    block->max_exit_speed_sqr  = block->nominal_speed_sqr;

    if (planner.count > 0) {
        uint8_t pi = (planner.head==0)?(PLANNER_BUFFER_SIZE-1):(planner.head-1);
        PlannerBlock_t *prev = &planner.blocks[pi];
        float cos_theta = 0.0f;
        for (int i=0;i<CNC_NUM_AXES;i++) cos_theta -= prev->unit_vec[i]*block->unit_vec[i];
        if (cos_theta < 0.95f) {
            float sth2 = sqrtf(0.5f*(1.0f-cos_theta));
            if (sth2 > 0.001f) {
                float js = CNC_min(block->nominal_speed, prev->nominal_speed);
                float vj = sqrtf(block->acceleration * PLANNER_JUNCTION_DEVIATION * sth2 / (1.0f-sth2));
                if (vj < js) js = vj;
                block->entry_speed_sqr = js*js;
            } else block->entry_speed_sqr = block->nominal_speed_sqr;
        } else {
            block->entry_speed_sqr = CNC_min(block->nominal_speed_sqr, prev->nominal_speed_sqr);
        }
    } else {
        float ms = PLANNER_MIN_SPEED;
        block->entry_speed_sqr = ms*ms;
    }
    block->exit_speed_sqr     = PLANNER_MIN_SPEED * PLANNER_MIN_SPEED;
    block->recalculate_flag   = 1;
    block->nominal_length_flag= 0;

    planner.head  = (planner.head+1) % PLANNER_BUFFER_SIZE;
    planner.count++;
    CNC_Planner_Recalculate();
    return true;
}

void CNC_Planner_Recalculate(void) {
    if (planner.count < 2) return;
    uint8_t idx = planner.head;
    for (uint8_t i=0;i<planner.count;i++) {
        idx = (idx==0)?(PLANNER_BUFFER_SIZE-1):(idx-1);
        PlannerBlock_t *cur = &planner.blocks[idx];
        if (i==0) cur->exit_speed_sqr = PLANNER_MIN_SPEED*PLANNER_MIN_SPEED;
        float me_sqr = cur->exit_speed_sqr + 2.0f*cur->acceleration*cur->distance;
        if (me_sqr < cur->entry_speed_sqr) { cur->entry_speed_sqr = me_sqr; cur->recalculate_flag=1; }
        if (i < planner.count-1) {
            uint8_t pi = (idx==0)?(PLANNER_BUFFER_SIZE-1):(idx-1);
            PlannerBlock_t *prev = &planner.blocks[pi];
            if (cur->entry_speed_sqr < prev->exit_speed_sqr) {
                prev->exit_speed_sqr = cur->entry_speed_sqr; prev->recalculate_flag=1;
            }
        }
    }
    idx = planner.tail;
    for (uint8_t i=0;i<planner.count;i++) {
        PlannerBlock_t *cur = &planner.blocks[idx];
        float mex_sqr = cur->entry_speed_sqr + 2.0f*cur->acceleration*cur->distance;
        if (mex_sqr < cur->exit_speed_sqr) cur->exit_speed_sqr = mex_sqr;
        if (cur->exit_speed_sqr > cur->nominal_speed_sqr) {
            cur->exit_speed_sqr = cur->nominal_speed_sqr; cur->nominal_length_flag=1;
        }
        idx = (idx+1) % PLANNER_BUFFER_SIZE;
    }
}

PlannerBlock_t* CNC_Planner_GetCurrentBlock(void) {
    if (CNC_Planner_IsBufferEmpty()) return NULL;
    return &planner.blocks[planner.tail];
}

void CNC_Planner_DiscardCurrentBlock(void) {
    if (!CNC_Planner_IsBufferEmpty()) {
        PlannerBlock_t *b = &planner.blocks[planner.tail];
        for (int i=0;i<CNC_NUM_AXES;i++) planner.previous_unit_vec[i] = b->unit_vec[i];
        planner.previous_nominal_speed = b->nominal_speed;
        planner.tail  = (planner.tail+1) % PLANNER_BUFFER_SIZE;
        planner.count--;
    }
}

bool    CNC_Planner_IsBufferEmpty(void) { return planner.count == 0; }
bool    CNC_Planner_IsBufferFull(void)  { return planner.count >= PLANNER_BUFFER_SIZE; }
uint8_t CNC_Planner_BlocksAvailable(void){ return PLANNER_BUFFER_SIZE - planner.count; }
uint8_t CNC_Planner_BlocksCount(void)   { return planner.count; }
void    CNC_Planner_Clear(void)         { planner.head=planner.tail=planner.count=0; }

#endif /* CNC_USE_PLANNER */

/*============================================================================
 *                    MOTION CONTROL
 *============================================================================*/

static void CNC_Move_Internal(float *target, float feedrate, bool rapid, bool no_accel)
{
    while (cnc_machine.is_moving) __NOP();

    if (cnc_machine.state == CNC_STATE_ALARM ||
        cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) {
        CNC_UART_PutString("[ALARM - CLEAR FIRST]\r\n"); return;
    }
    if (!cnc_machine.motors_enabled) CNC_EnableMotors(true);

    float delta[CNC_NUM_AXES];
    float unit_vec[CNC_NUM_AXES] = {0};
    int64_t new_bt = 0;
    int     dom    = 0;
    float   dist   = 0.0f;

    for (int i = 0; i < CNC_NUM_AXES; i++) {
        delta[i] = target[i] - cnc_machine.position[i];
        cnc_machine.target[i]              = target[i];
        cnc_machine.logical_delta[i]       = delta[i];
        cnc_machine.logical_steps_total[i] = (int32_t)fabsf(delta[i]*CNC_SafeStepsPerMM(i));

        int64_t asteps = (int64_t)roundf(fabsf(delta[i]*CNC_SafeStepsPerMM(i)));
        if (asteps > MAX_STEPS_PER_MOVE) { CNC_UART_PutString("[ERROR: Move too long]\r\n"); return; }

        bresenham_steps[i]         = asteps;
        cnc_machine.step_count[i]  = (int32_t)asteps;
        cnc_machine.steps_total[i] = (int32_t)asteps;
        if (asteps > new_bt) { new_bt = asteps; dom = i; }

        if      (delta[i] >  0.0001f) CNC_SetDirection(i,  1);
        else if (delta[i] < -0.0001f) CNC_SetDirection(i, -1);
        else cnc_machine.direction[i] = 0;

        dist += delta[i]*delta[i];
    }
    dist = sqrtf(dist);
    if (new_bt == 0) return;

    if (dist > 0.0001f)
        for (int i=0;i<CNC_NUM_AXES;i++) unit_vec[i] = delta[i]/dist;

    for (int i=0;i<CNC_NUM_AXES;i++) bresenham_counter[i] = new_bt / 2;

    __disable_irq();
    bresenham_total       = new_bt;
    step_events_remaining = new_bt;
    __enable_irq();

    float vmax_feed = CNC_ComputeVectorMaxSpeedMMs(unit_vec) * 60.0f;
    float eff_feed  = rapid ? vmax_feed : CNC_min(feedrate, vmax_feed);
    if (eff_feed < 1.0f) eff_feed = 1.0f;

    cnc_machine.dominant_axis = dom;
    cnc_machine.move_distance = dist;

    if (no_accel) {
        float spm_dom  = CNC_SafeStepsPerMM(dom);
        uint32_t freq  = (uint32_t)((eff_feed/60.0f)*spm_dom);
        if (freq < 10)     freq = 10;
        if (freq > 500000) freq = 500000;
        uint32_t period = 1000000UL / freq;
        if (period < MIN_TIMER_PERIOD_US) period = MIN_TIMER_PERIOD_US;
        if (period > MAX_TIMER_PERIOD_US) period = MAX_TIMER_PERIOD_US;
        current_step_period = CNC_Step_CalcDuty(period);
#if CNC_USE_ACCELERATION
        cnc_machine.accel.phase        = ACCEL_PHASE_CRUISE;
        cnc_machine.accel.current_speed= eff_feed/60.0f;
        cnc_machine.accel.target_speed = eff_feed/60.0f;
        cnc_machine.accel.needs_recalc = false;
#endif
    } else {
#if CNC_USE_ACCELERATION
        float vaccel = CNC_ComputeVectorMaxAccelMMs2(unit_vec);
        CNC_Accel_Plan(dist, eff_feed, vaccel);
#else
        float spm_dom  = CNC_SafeStepsPerMM(dom);
        uint32_t freq  = (uint32_t)((eff_feed/60.0f)*spm_dom);
        if (freq < 10)     freq = 10;
        if (freq > 500000) freq = 500000;
        CNC_Timer_SetPeriod(1000000UL / freq);
#endif
    }

    cnc_machine.is_moving = true;
    CNC_Timer_Start();
}

void CNC_Move(float *target, float feedrate, bool rapid) {
    if (cnc_machine.state == CNC_STATE_IDLE ||
        cnc_machine.state == CNC_STATE_RUN)
        cnc_machine.state = CNC_STATE_RUN;
    CNC_Move_Internal(target, feedrate, rapid, false);
}

void CNC_Move_NoAccel(float *target, float feedrate) {
    CNC_Move_Internal(target, feedrate, false, true);
}

void CNC_MoveWithCompensation(float *target, float feedrate, bool rapid) {
    CNC_Move(target, feedrate, rapid);
}

bool CNC_IsMoving(void) { return cnc_machine.is_moving; }

void CNC_Stop(void) {
    CNC_Timer_Stop();
    cnc_machine.is_moving = false;
    __disable_irq();
    step_events_remaining = 0;
    bresenham_total       = 0;
    __enable_irq();
    for (int i=0;i<CNC_NUM_AXES;i++) cnc_machine.step_count[i] = 0;
#if CNC_USE_ACCELERATION
    cnc_machine.accel.mm_per_step     = 0;
    cnc_machine.accel.needs_recalc    = false;
    cnc_machine.accel.last_update_tick= 0;
    cnc_machine.accel.precomp_period  = 0;
#endif
    if (cnc_machine.state == CNC_STATE_GOWDELAY)     cnc_machine.gowdelay.phase     = GOWDELAY_IDLE;
    if (cnc_machine.state == CNC_STATE_GOWDELAY_RECT) cnc_machine.gowdelay_rect.phase = GWDRECT_IDLE;
    if (cnc_machine.state != CNC_STATE_ALARM &&
        cnc_machine.state != CNC_STATE_LIMIT_TRIGGERED)
        cnc_machine.state = CNC_STATE_IDLE;
#if CNC_USE_COMMAND_QUEUE
    CNC_Queue_Clear();
#endif
#if CNC_USE_PLANNER
    CNC_Planner_Clear();
#endif
}

void CNC_WaitComplete(void) {
    while (cnc_machine.is_moving) {
#if CNC_USE_LIMIT_SWITCH
        CNC_Limit_CheckAll();
#endif
        __NOP();
    }
}

void CNC_SetPosition(float *pos) {
    for (int i=0;i<CNC_NUM_AXES;i++) cnc_machine.position[i] = pos[i];
}

void CNC_SetAxisPosition(uint8_t axis, float value) {
    if (axis < CNC_NUM_AXES) cnc_machine.position[axis] = value;
}

void CNC_GetPosition(float *pos) {
    for (int i=0;i<CNC_NUM_AXES;i++) pos[i] = cnc_machine.position[i];
}

/*============================================================================
 *                    GUI PROTOCOL IMPLEMENTATION
 *============================================================================*/

#if CNC_USE_GUI_PROTOCOL

/* ---- helpers ---- */
static inline int32_t GUI_MM2Pulse(float mm) { return (int32_t)roundf(mm * GUI_PULSE_PER_MM); }
static inline float   GUI_Pulse2MM(int32_t p) { return (float)p / GUI_PULSE_PER_MM; }

static bool GUI_FindValue(const char *str, char key, float *out)
{
    if (!str || !out) return false;
    if (key >= 'a' && key <= 'z') key -= 32;
    const char *p = str;
    while (*p) {
        char c = *p;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c == key) {
            bool ok = (p==str) || (*(p-1)==' ') || (*(p-1)=='\t');
            if (ok) {
                const char *num = p+1;
                if (*num=='-'||*num=='+'||*num=='.'||(*num>='0'&&*num<='9')) {
                    char *ep;
                    float val = strtof(num, &ep);
                    if (ep != num) { *out = val; return true; }
                }
            }
        }
        p++;
    }
    return false;
}

static int8_t GUI_PresetIndex(char name) {
    switch (name) {
        case 'A': return 0; case 'B': return 1;
        case 'C': return 2; case 'D': return 3;
        default:  return -1;
    }
}

/* ---- state string ---- */
const char* CNC_GUI_GetStateString(void)
{
    if (cnc_machine.state == CNC_STATE_ALARM ||
        cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) return "Alarm";

    switch (cnc_machine.gui_action) {
        case GUI_ACTION_STOP:    return "STOP";
        case GUI_ACTION_GOTO_A:  return "GoTo A";
        case GUI_ACTION_GOTO_B:  return "GoTo B";
        case GUI_ACTION_GOTO_C:  return "GoTo C";
        case GUI_ACTION_GOTO_D:  return "GoTo D";
        case GUI_ACTION_GCODE_MOVE: return "Run GCode";
        default: break;
    }
    switch (cnc_machine.state) {
        case CNC_STATE_RUN:     return "Run GCode";
        case CNC_STATE_HOMING:  return "Homing";
        case CNC_STATE_HOLD:    return "Hold";
        case CNC_STATE_GOWDELAY:     return "GoWDelay";
        case CNC_STATE_GOWDELAY_RECT:return "GoWDelayRect";
        default: return "Idle";
    }
}

/* ---- status report ---- */
void CNC_GUI_SendStatus(void)
{
    /* Auto-clear GOTO action when motion done */
    if (!cnc_machine.is_moving &&
        (cnc_machine.gui_action == GUI_ACTION_GOTO_A ||
         cnc_machine.gui_action == GUI_ACTION_GOTO_B ||
         cnc_machine.gui_action == GUI_ACTION_GOTO_C ||
         cnc_machine.gui_action == GUI_ACTION_GOTO_D ||
         cnc_machine.gui_action == GUI_ACTION_GCODE_MOVE) &&
        cnc_machine.state == CNC_STATE_IDLE) {
        cnc_machine.gui_action = GUI_ACTION_IDLE;
    }

    CNC_UART_PutChar('<');
    CNC_UART_PutString(CNC_GUI_GetStateString());
    CNC_UART_PutString("|WPos:");
    CNC_UART_PutFloat(cnc_machine.position[0], 3); CNC_UART_PutChar(',');
    CNC_UART_PutFloat(cnc_machine.position[1], 3); CNC_UART_PutChar(',');

    /* Z - check sentinel */
    if (cnc_machine.position[2] <= (GUI_SENTINEL_MM + 0.0005f))
        CNC_UART_PutString("-9.999");
    else
        CNC_UART_PutFloat(cnc_machine.position[2], 3);
    CNC_UART_PutChar(',');

    /* V - check sentinel */
    if (cnc_machine.position[3] <= (GUI_SENTINEL_MM + 0.0005f))
        CNC_UART_PutString("-9.999");
    else
        CNC_UART_PutFloat(cnc_machine.position[3], 3);

    CNC_UART_PutString(">\r\n");
}

/* ---- STOP ---- */
void CNC_GUI_Handle_STOP(void)
{
    CNC_Stop();
    cnc_machine.gui_action = GUI_ACTION_STOP;
    CNC_UART_PutString("STOPPED\r\n");
}

/* ---- SET ---- */
void CNC_GUI_Handle_SET(const char *params)
{
    while (*params==' '||*params=='\t') params++;
    if (!params[0]) { CNC_UART_PutString("Error: Invalid position name.\r\n"); return; }

    char first = params[0];

    /* Axis reset: SET X/Y/Z/V alone */
    if (first=='X'||first=='Y'||first=='Z'||first=='V') {
        const char *rest = params+1;
        while (*rest==' '||*rest=='\t') rest++;
        if (*rest=='\0') {
            uint8_t ax = (first=='X')?0:(first=='Y')?1:(first=='Z')?2:3;
            CNC_SetAxisPosition(ax, 0.0f);
            CNC_UART_PutString("Reset "); CNC_UART_PutChar(first);
            CNC_UART_PutString(" OK\r\n");
            return;
        }
    }

    /* Preset SET A/B/C/D */
    int8_t idx = GUI_PresetIndex(first);
    if (idx < 0) { CNC_UART_PutString("Error: Invalid position name.\r\n"); return; }

    const char *rest = params+1;
    float val;
    if (GUI_FindValue(rest,'X',&val)) gui_proto.presets[idx].axis[0] = GUI_MM2Pulse(val);
    if (GUI_FindValue(rest,'Y',&val)) gui_proto.presets[idx].axis[1] = GUI_MM2Pulse(val);
    if (GUI_FindValue(rest,'Z',&val)) gui_proto.presets[idx].axis[2] = GUI_MM2Pulse(val);
    if (GUI_FindValue(rest,'V',&val)) gui_proto.presets[idx].axis[3] = GUI_MM2Pulse(val);

    CNC_UART_PutString("Position set OK\r\n");
}

/* ---- RETURN ---- */
void CNC_GUI_Handle_RETURN(const char *params)
{
    while (*params==' '||*params=='\t') params++;
    if (!params[0]) { CNC_UART_PutString("Error: Invalid position for RETURN command.\r\n"); return; }

    int8_t idx = GUI_PresetIndex(params[0]);
    if (idx < 0) { CNC_UART_PutString("Error: Invalid position for RETURN command.\r\n"); return; }

    GUI_Position_t *p = &gui_proto.presets[idx];
    char name = (char)('A' + idx);

    CNC_UART_PutChar(name); CNC_UART_PutChar(' ');

    const char axis_names[CNC_NUM_AXES] = {'X','Y','Z','V'};
    for (int i=0;i<CNC_NUM_AXES;i++) {
        CNC_UART_PutChar(axis_names[i]);
        if (p->axis[i] == GUI_SENTINEL_PULSE)
            CNC_UART_PutString("-9.999");
        else
            CNC_UART_PutFloat(GUI_Pulse2MM(p->axis[i]), 3);
    }
    CNC_UART_PutString("\r\n");
}

/* ---- GOTO ---- */
void CNC_GUI_Handle_GOTO(const char *params)
{
    while (*params==' '||*params=='\t') params++;
    if (!params[0]) { CNC_UART_PutString("Error: Invalid Go To Position.\r\n"); return; }

    int8_t idx = GUI_PresetIndex(params[0]);
    if (idx < 0) { CNC_UART_PutString("Error: Invalid Go To Position.\r\n"); return; }

    GUI_Position_t *preset = &gui_proto.presets[idx];

    /* Build target */
    float target[CNC_NUM_AXES];
    CNC_GetPosition(target);

    bool has_motion = false;
    float feedrate  = DEFAULT_FEEDRATE;

    for (int i=0;i<CNC_NUM_AXES;i++) {
        if (preset->axis[i] != GUI_SENTINEL_PULSE) {
            target[i]  = GUI_Pulse2MM(preset->axis[i]);
            has_motion = true;
            /* Take the minimum max_feedrate of active axes */
            if (cnc_config.max_feedrate[i] < feedrate)
                feedrate = cnc_config.max_feedrate[i];
        }
    }

    /* Set action */
    cnc_machine.gui_action = (GUI_Action_t)(GUI_ACTION_GOTO_A + idx);

    /* Response first */
    CNC_UART_PutString("Go TO Position OK\r\n");

    if (!has_motion) return;

    /* Save previous state, force GOTO state for ISR */
    cnc_machine.state = CNC_STATE_GOTO;

    /* Use rapid move (G0 semantics) */
    CNC_Move_Internal(target, feedrate, true, false);
}

/* ---- GUI Init ---- */
void CNC_GUI_Init(void)
{
    memset(&gui_proto, 0, sizeof(gui_proto));

    /* Default presets */
    gui_proto.presets[0].axis[0] = GUI_MM2Pulse(GUI_PRESET_A_X_MM);
    gui_proto.presets[0].axis[1] = GUI_MM2Pulse(GUI_PRESET_A_Y_MM);
    gui_proto.presets[0].axis[2] = GUI_PRESET_A_Z_PULSE;
    gui_proto.presets[0].axis[3] = GUI_PRESET_A_V_PULSE;

    gui_proto.presets[1].axis[0] = GUI_MM2Pulse(GUI_PRESET_B_X_MM);
    gui_proto.presets[1].axis[1] = GUI_MM2Pulse(GUI_PRESET_B_Y_MM);
    gui_proto.presets[1].axis[2] = GUI_PRESET_B_Z_PULSE;
    gui_proto.presets[1].axis[3] = GUI_PRESET_B_V_PULSE;

    gui_proto.presets[2].axis[0] = GUI_MM2Pulse(GUI_PRESET_C_X_MM);
    gui_proto.presets[2].axis[1] = GUI_MM2Pulse(GUI_PRESET_C_Y_MM);
    gui_proto.presets[2].axis[2] = GUI_PRESET_C_Z_PULSE;
    gui_proto.presets[2].axis[3] = GUI_PRESET_C_V_PULSE;

    gui_proto.presets[3].axis[0] = GUI_MM2Pulse(GUI_PRESET_D_X_MM);
    gui_proto.presets[3].axis[1] = GUI_MM2Pulse(GUI_PRESET_D_Y_MM);
    gui_proto.presets[3].axis[2] = GUI_PRESET_D_Z_PULSE;
    gui_proto.presets[3].axis[3] = GUI_PRESET_D_V_PULSE;

    gui_proto.initialized = true;
}

#endif /* CNC_USE_GUI_PROTOCOL */
/*============================================================================
 *                    UART BASIC
 *============================================================================*/

static void CNC_UART_Init(void) {
#if CNC_UART_APB2
    RCC->APB2ENR |= CNC_UART_RCC;
#else
    RCC->APB1ENR |= CNC_UART_RCC;
#endif

#if (CNC_UART_SELECT == 1)
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 |GPIO_CRH_MODE9);
    GPIOA->CRH |=  (GPIO_CRH_CNF9_1|GPIO_CRH_MODE9);
    GPIOA->CRH &= ~(GPIO_CRH_CNF10|GPIO_CRH_MODE10);
    GPIOA->CRH |=   GPIO_CRH_CNF10_0;
#elif (CNC_UART_SELECT == 2)
    GPIOA->CRL &= ~(GPIO_CRL_CNF2|GPIO_CRL_MODE2);
    GPIOA->CRL |=  (GPIO_CRL_CNF2_1|GPIO_CRL_MODE2);
    GPIOA->CRL &= ~(GPIO_CRL_CNF3|GPIO_CRL_MODE3);
    GPIOA->CRL |=   GPIO_CRL_CNF3_0;
#elif (CNC_UART_SELECT == 3)
    GPIOB->CRH &= ~(GPIO_CRH_CNF10|GPIO_CRH_MODE10);
    GPIOB->CRH |=  (GPIO_CRH_CNF10_1|GPIO_CRH_MODE10);
    GPIOB->CRH &= ~(GPIO_CRH_CNF11|GPIO_CRH_MODE11);
    GPIOB->CRH |=   GPIO_CRH_CNF11_0;
#endif

    CNC_UART->CR1 = CNC_UART->CR2 = CNC_UART->CR3 = 0;
#if CNC_UART_APB2
    CNC_UART->BRR = CNC_SYSTEM_CLOCK / CNC_UART_BAUDRATE;
#else
    CNC_UART->BRR = (CNC_SYSTEM_CLOCK/2) / CNC_UART_BAUDRATE;
#endif
    CNC_UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

#if !CNC_USE_DMA_UART
    CNC_UART->CR1 |= USART_CR1_RXNEIE;
    NVIC_SetPriority(CNC_UART_IRQn, 2);
    NVIC_EnableIRQ(CNC_UART_IRQn);
#endif
    cnc_uart_rx.head = cnc_uart_rx.tail = cnc_uart_rx.count = 0;
}

void CNC_UART_PutChar(char c) { while(!(CNC_UART->SR&USART_SR_TXE)); CNC_UART->DR=c; }
void CNC_UART_PutString(const char *s) { while(*s) CNC_UART_PutChar(*s++); }

void CNC_UART_PutInt(int32_t num) {
    char buf[12]; int i=0; bool neg=false;
    if (num<0){neg=true;num=-num;}
    if (num==0){CNC_UART_PutChar('0');return;}
    while(num>0){buf[i++]='0'+(num%10);num/=10;}
    if(neg)CNC_UART_PutChar('-');
    while(i>0)CNC_UART_PutChar(buf[--i]);
}

void CNC_UART_PutFloat(float num, uint8_t decimals) {
    if(num<0){CNC_UART_PutChar('-');num=-num;}
    int32_t ip=(int32_t)num; CNC_UART_PutInt(ip); CNC_UART_PutChar('.');
    float frac=num-(float)ip;
    for(uint8_t i=0;i<decimals;i++){frac*=10;CNC_UART_PutChar('0'+(int)frac%10);}
}

bool CNC_UART_Available(void) {
#if CNC_USE_DMA_UART
    return CNC_UART_DMA_Available() > 0;
#else
    return cnc_uart_rx.count > 0;
#endif
}

char CNC_UART_GetChar(void) {
#if CNC_USE_DMA_UART
    return (char)CNC_UART_DMA_Read();
#else
    if(!cnc_uart_rx.count) return 0;
    char c=cnc_uart_rx.buffer[cnc_uart_rx.tail];
    cnc_uart_rx.tail=(cnc_uart_rx.tail+1)%UART_BUFFER_SIZE;
    cnc_uart_rx.count--; return c;
#endif
}

bool CNC_UART_GetLine(char *line, uint16_t max_len) {
    static char temp[GCODE_LINE_SIZE];
    static uint16_t idx = 0;
    while (CNC_UART_Available()) {
        char c = CNC_UART_GetChar();
        if (c=='\r') continue;
        if (c=='\n') {
            temp[idx]='\0';
            strncpy(line,temp,max_len);
            line[max_len-1]='\0';
            idx=0; return true;
        }
        if (idx < GCODE_LINE_SIZE-1) {
            /* Uppercase for non-GUI lines is done in parser */
            temp[idx++] = c;
        }
    }
    return false;
}

void CNC_UART_SendOK(void)              { CNC_UART_PutString("OK\r\n"); }
void CNC_UART_SendError(uint8_t code)   { CNC_UART_PutString("ERROR:"); CNC_UART_PutInt(code); CNC_UART_PutString("\r\n"); }

#if !CNC_USE_DMA_UART
void CNC_UART_IRQHandler(void) {
    if (CNC_UART->SR & USART_SR_RXNE) {
        uint8_t d = CNC_UART->DR;
        if (cnc_uart_rx.count < UART_BUFFER_SIZE) {
            cnc_uart_rx.buffer[cnc_uart_rx.head] = d;
            cnc_uart_rx.head=(cnc_uart_rx.head+1)%UART_BUFFER_SIZE;
            cnc_uart_rx.count++;
        }
    }
    if (CNC_UART->SR & USART_SR_ORE) { volatile uint32_t t=CNC_UART->SR; t=CNC_UART->DR; (void)t; }
}
#endif

/*============================================================================
 *                    GOWDELAY
 *============================================================================*/

void CNC_GoWDelay_Start(float *target, float step_distance, uint32_t delay_ms, float feedrate)
{
    if (step_distance <= 0.001f) { CNC_UART_PutString("[ERROR: D must be > 0]\r\n"); return; }
    if (cnc_machine.state != CNC_STATE_IDLE) { CNC_UART_PutString("[ERROR: Machine busy]\r\n"); return; }

    GoWDelayState_t *gwd = &cnc_machine.gowdelay;
    CNC_GetPosition(gwd->start_pos);
    for (int i=0;i<CNC_NUM_AXES;i++) gwd->target_pos[i] = target[i];

    float delta[CNC_NUM_AXES];
    gwd->total_distance = 0;
    for (int i=0;i<CNC_NUM_AXES;i++) { delta[i]=target[i]-gwd->start_pos[i]; gwd->total_distance+=delta[i]*delta[i]; }
    gwd->total_distance = sqrtf(gwd->total_distance);
    if (gwd->total_distance < 0.001f) { CNC_UART_PutString("[NO MOVEMENT]\r\n"); return; }

    for (int i=0;i<CNC_NUM_AXES;i++) gwd->unit_vector[i] = delta[i]/gwd->total_distance;

    uint32_t full_steps = (uint32_t)(gwd->total_distance / step_distance);
    float    remainder  = gwd->total_distance - (full_steps * step_distance);
    gwd->total_steps    = (remainder > 0.001f) ? full_steps+1 : (full_steps?full_steps:1);
    gwd->step_distance  = step_distance;
    gwd->delay_ms       = delay_ms;
    gwd->feedrate       = (feedrate>0)?feedrate:current_feedrate;
    gwd->current_step   = 0;
    gwd->distance_moved = 0;
    gwd->delay_start_tick = 0;

    cnc_machine.state = CNC_STATE_GOWDELAY;
    gwd->phase = GOWDELAY_MOVING;

    CNC_UART_PutString("[GOWDELAY: "); CNC_UART_PutInt(gwd->total_steps);
    CNC_UART_PutString(" steps, D="); CNC_UART_PutFloat(step_distance,2);
    CNC_UART_PutString("mm, W="); CNC_UART_PutInt(delay_ms);
    CNC_UART_PutString("ms]\r\n");
    CNC_EnableMotors(true);

    float next_pos[CNC_NUM_AXES];
    float sd = CNC_min(gwd->step_distance, gwd->total_distance);
    gwd->distance_moved = sd;
    for (int i=0;i<CNC_NUM_AXES;i++) next_pos[i] = gwd->start_pos[i] + gwd->unit_vector[i]*gwd->distance_moved;
    CNC_Move_NoAccel(next_pos, gwd->feedrate);
}

static void CNC_GoWDelay_NextStep(void) {
    GoWDelayState_t *gwd = &cnc_machine.gowdelay;
    float next_pos[CNC_NUM_AXES];
    float remaining = gwd->total_distance - gwd->distance_moved;

    if (gwd->current_step >= gwd->total_steps-1 || remaining <= gwd->step_distance+0.001f) {
        for (int i=0;i<CNC_NUM_AXES;i++) next_pos[i] = gwd->target_pos[i];
        gwd->distance_moved = gwd->total_distance;
    } else {
        float sd = CNC_min(gwd->step_distance, remaining);
        gwd->distance_moved += sd;
        for (int i=0;i<CNC_NUM_AXES;i++) next_pos[i] = gwd->start_pos[i] + gwd->unit_vector[i]*gwd->distance_moved;
    }
    gwd->phase = GOWDELAY_MOVING;
    CNC_Move_NoAccel(next_pos, gwd->feedrate);
}

void CNC_GoWDelay_Process(void) {
    if (cnc_machine.state != CNC_STATE_GOWDELAY) return;
    GoWDelayState_t *gwd = &cnc_machine.gowdelay;
    switch (gwd->phase) {
        case GOWDELAY_MOVING:
            if (!cnc_machine.is_moving) {
                gwd->current_step++;
                if (gwd->current_step >= gwd->total_steps) {
                    gwd->phase = GOWDELAY_COMPLETE;
                    cnc_machine.state = CNC_STATE_IDLE;
                    CNC_UART_PutString("[GOWDELAY COMPLETE]\r\n"); return;
                }
                if (gwd->delay_ms > 0) { gwd->phase=GOWDELAY_WAITING; gwd->delay_start_tick=CNC_GetTick(); }
                else CNC_GoWDelay_NextStep();
            }
            break;
        case GOWDELAY_WAITING:
            if ((CNC_GetTick()-gwd->delay_start_tick) >= gwd->delay_ms) CNC_GoWDelay_NextStep();
            break;
        default:
            if (cnc_machine.state==CNC_STATE_GOWDELAY) cnc_machine.state=CNC_STATE_IDLE;
            break;
    }
}

void CNC_GoWDelay_Stop(void) {
    if (cnc_machine.state==CNC_STATE_GOWDELAY) {
        CNC_Stop(); cnc_machine.gowdelay.phase=GOWDELAY_IDLE; cnc_machine.state=CNC_STATE_IDLE;
        CNC_UART_PutString("[GOWDELAY ABORTED]\r\n");
    }
}
bool CNC_GoWDelay_IsActive(void) { return cnc_machine.state==CNC_STATE_GOWDELAY; }

/*============================================================================
 *                    GOWDELAYRECT
 *============================================================================*/

void CNC_GoWDelayRect_Start(float size_x, float size_y, float step_distance, uint32_t delay_ms, float feedrate)
{
    if (step_distance<=0.001f){CNC_UART_PutString("[ERROR: D>0]\r\n");return;}
    if (fabsf(size_x)<0.001f||fabsf(size_y)<0.001f){CNC_UART_PutString("[ERROR: X,Y!=0]\r\n");return;}
    if (cnc_machine.state!=CNC_STATE_IDLE){CNC_UART_PutString("[ERROR: Busy]\r\n");return;}

    GoWDelayRectState_t *gwr = &cnc_machine.gowdelay_rect;
    CNC_GetPosition(gwr->start_pos);
    gwr->rect_size_x = size_x; gwr->rect_size_y = size_y;
    gwr->step_distance = step_distance; gwr->delay_ms = delay_ms;
    gwr->feedrate = (feedrate>0)?feedrate:current_feedrate;

    float ax = fabsf(size_x), ay = fabsf(size_y);
    uint32_t fr = (uint32_t)(ay/step_distance);
    gwr->total_rows = (ay-(fr*step_distance)>0.001f)?fr+1:(fr?fr:1);
    uint32_t fsx = (uint32_t)(ax/step_distance);
    gwr->steps_per_row = (ax-(fsx*step_distance)>0.001f)?fsx+1:(fsx?fsx:1);
    gwr->total_steps   = gwr->steps_per_row * gwr->total_rows;

    gwr->current_x = gwr->current_y = 0;
    gwr->x_direction = (size_x>=0)?1:-1;
    gwr->current_row = gwr->current_step_in_row = gwr->completed_steps = 0;
    gwr->delay_start_tick = 0;
    cnc_machine.state = CNC_STATE_GOWDELAY_RECT;
    gwr->phase = GWDRECT_MOVE_X;
    CNC_EnableMotors(true);

    CNC_UART_PutString("[GOWDELAYRECT "); CNC_UART_PutFloat(size_x,1);
    CNC_UART_PutString("x"); CNC_UART_PutFloat(size_y,1);
    CNC_UART_PutString("mm, "); CNC_UART_PutInt(gwr->total_steps);
    CNC_UART_PutString(" steps]\r\n");

    CNC_GoWDelayRect_NextStepX();
}

static void CNC_GoWDelayRect_NextStepX(void) {
    GoWDelayRectState_t *gwr = &cnc_machine.gowdelay_rect;
    float ax = fabsf(gwr->rect_size_x);
    if (gwr->rect_size_x >= 0) {
        if (gwr->x_direction>0){ gwr->current_x+=gwr->step_distance; if(gwr->current_x>ax) gwr->current_x=ax; }
        else                   { gwr->current_x-=gwr->step_distance; if(gwr->current_x<0)  gwr->current_x=0; }
    } else {
        if (gwr->x_direction<0){ gwr->current_x-=gwr->step_distance; if(gwr->current_x<-ax) gwr->current_x=-ax; }
        else                   { gwr->current_x+=gwr->step_distance; if(gwr->current_x>0)   gwr->current_x=0; }
    }
    float next_pos[CNC_NUM_AXES];
    next_pos[0]=gwr->start_pos[0]+gwr->current_x;
    next_pos[1]=gwr->start_pos[1]+gwr->current_y;
    next_pos[2]=gwr->start_pos[2]; next_pos[3]=gwr->start_pos[3];
    gwr->phase=GWDRECT_MOVE_X;
    CNC_Move_NoAccel(next_pos, gwr->feedrate);
}

static void CNC_GoWDelayRect_NextStepY(void) {
    GoWDelayRectState_t *gwr = &cnc_machine.gowdelay_rect;
    float ay = fabsf(gwr->rect_size_y);
    if (gwr->rect_size_y>=0) {
        float rem=ay-gwr->current_y; float sy=CNC_min(gwr->step_distance,rem);
        gwr->current_y+=sy; if(gwr->current_y>ay) gwr->current_y=ay;
    } else {
        float rem=ay+gwr->current_y; float sy=CNC_min(gwr->step_distance,rem);
        gwr->current_y-=sy; if(gwr->current_y<-ay) gwr->current_y=-ay;
    }
    float next_pos[CNC_NUM_AXES];
    next_pos[0]=gwr->start_pos[0]+gwr->current_x;
    next_pos[1]=gwr->start_pos[1]+gwr->current_y;
    next_pos[2]=gwr->start_pos[2]; next_pos[3]=gwr->start_pos[3];
    gwr->phase=GWDRECT_MOVE_Y;
    CNC_Move_NoAccel(next_pos, gwr->feedrate);
}

void CNC_GoWDelayRect_Process(void) {
    if (cnc_machine.state!=CNC_STATE_GOWDELAY_RECT) return;
    GoWDelayRectState_t *gwr = &cnc_machine.gowdelay_rect;
    float ax=fabsf(gwr->rect_size_x), ay=fabsf(gwr->rect_size_y);

    switch (gwr->phase) {
        case GWDRECT_MOVE_X:
            if (!cnc_machine.is_moving) {
                gwr->current_step_in_row++; gwr->completed_steps++;
                bool row_done = (gwr->rect_size_x>=0)
                    ? ((gwr->x_direction>0)?(gwr->current_x>=ax-0.001f):(gwr->current_x<=0.001f))
                    : ((gwr->x_direction<0)?(gwr->current_x<=-ax+0.001f):(gwr->current_x>=-0.001f));

                if (row_done) {
                    bool all_done = (gwr->rect_size_y>=0)
                        ? (gwr->current_y>=ay-0.001f)
                        : (gwr->current_y<=-ay+0.001f);
                    if (all_done) {
                        gwr->phase=GWDRECT_COMPLETE; cnc_machine.state=CNC_STATE_IDLE;
                        CNC_UART_PutString("[GOWDELAYRECT COMPLETE]\r\n"); return;
                    }
                    gwr->x_direction=-gwr->x_direction; gwr->current_row++; gwr->current_step_in_row=0;
                    if (gwr->delay_ms>0){gwr->phase=GWDRECT_WAIT_X;gwr->delay_start_tick=CNC_GetTick();}
                    else CNC_GoWDelayRect_NextStepY();
                } else {
                    if (gwr->delay_ms>0){gwr->phase=GWDRECT_WAIT_X;gwr->delay_start_tick=CNC_GetTick();}
                    else CNC_GoWDelayRect_NextStepX();
                }
            }
            break;
        case GWDRECT_WAIT_X:
            if ((CNC_GetTick()-gwr->delay_start_tick)>=gwr->delay_ms) {
                if (gwr->current_step_in_row==0) CNC_GoWDelayRect_NextStepY();
                else CNC_GoWDelayRect_NextStepX();
            }
            break;
        case GWDRECT_MOVE_Y:
            if (!cnc_machine.is_moving) {
                if (gwr->delay_ms>0){gwr->phase=GWDRECT_WAIT_Y;gwr->delay_start_tick=CNC_GetTick();}
                else CNC_GoWDelayRect_NextStepX();
            }
            break;
        case GWDRECT_WAIT_Y:
            if ((CNC_GetTick()-gwr->delay_start_tick)>=gwr->delay_ms) CNC_GoWDelayRect_NextStepX();
            break;
        default:
            if (cnc_machine.state==CNC_STATE_GOWDELAY_RECT) cnc_machine.state=CNC_STATE_IDLE;
            break;
    }
}

void CNC_GoWDelayRect_Stop(void) {
    if (cnc_machine.state==CNC_STATE_GOWDELAY_RECT) {
        CNC_Stop(); cnc_machine.gowdelay_rect.phase=GWDRECT_IDLE; cnc_machine.state=CNC_STATE_IDLE;
        CNC_UART_PutString("[GOWDELAYRECT ABORTED]\r\n");
    }
}
bool CNC_GoWDelayRect_IsActive(void) { return cnc_machine.state==CNC_STATE_GOWDELAY_RECT; }

/*============================================================================
 *                    CONFIG
 *============================================================================*/

static float CNC_Config_ClampFloat(float v, float mn, float mx) {
    if (v!=v) return mn; if(v<mn) return mn; if(v>mx) return mx; return v;
}
static uint8_t CNC_Config_ClampMask4(float v) {
    int32_t i=(int32_t)v; if(i<0)i=0; if(i>15)i=15; return (uint8_t)i;
}
static bool CNC_Config_ClampBool(float v) { return v > 0.0f; }
static uint32_t CNC_Config_ClampU32(float v, uint32_t mn, uint32_t mx) {
    if(v!=v) return mn; if(v<(float)mn) return mn; if(v>(float)mx) return mx; return (uint32_t)v;
}

void CNC_Config_Set(const char *param, float value)
{
    if (!param || param[0]!='$') return;
    int idx = atoi(&param[1]);
    float applied = 0.0f;

    switch (idx) {
        case 2:   cnc_config.step_invert_mask  = CNC_Config_ClampMask4(value); applied=(float)cnc_config.step_invert_mask; break;
        case 3:   cnc_config.dir_invert_mask   = CNC_Config_ClampMask4(value); applied=(float)cnc_config.dir_invert_mask;  break;
        case 4:   cnc_config.enable_invert     = CNC_Config_ClampBool(value)?1:0; applied=(float)cnc_config.enable_invert; break;
        case 5:   cnc_config.limit_invert_mask = CNC_Config_ClampMask4(value); applied=(float)cnc_config.limit_invert_mask;break;
        case 20:  cnc_config.soft_limit_enabled= CNC_Config_ClampBool(value); applied=cnc_config.soft_limit_enabled?1:0;  break;
        case 21:  cnc_config.hard_limit_enabled= CNC_Config_ClampBool(value); applied=cnc_config.hard_limit_enabled?1:0;  break;
#if CNC_USE_HOMING
        case 22: cnc_config.homing_enabled   = CNC_Config_ClampBool(value);              applied=cnc_config.homing_enabled?1:0; break;
        case 23: cnc_config.homing_dir_mask  = CNC_Config_ClampMask4(value);             applied=(float)cnc_config.homing_dir_mask; break;
        case 24: cnc_config.homing_feed_rate = CNC_Config_ClampFloat(value,1,1000000);   applied=cnc_config.homing_feed_rate; break;
        case 25: cnc_config.homing_seek_rate = CNC_Config_ClampFloat(value,1,1000000);   applied=cnc_config.homing_seek_rate; break;
        case 26: cnc_config.homing_axis_mask = CNC_Config_ClampMask4(value);             applied=(float)cnc_config.homing_axis_mask; break;
        case 27: cnc_config.homing_pulloff   = CNC_Config_ClampFloat(value,0,1000);      applied=cnc_config.homing_pulloff; break;
        case 28: cnc_config.homing_cycle     = (uint8_t)CNC_Config_ClampU32(value,0,2); applied=(float)cnc_config.homing_cycle; break;
#endif
        case 32:
            cnc_config.auto_report_interval = CNC_Config_ClampU32(value,0,3600000);
            auto_report_counter = 0; applied=(float)cnc_config.auto_report_interval; break;

        case 100: cnc_config.steps_per_mm[0]=CNC_Config_ClampFloat(value,MIN_STEPS_PER_MM,MAX_STEPS_PER_MM); applied=cnc_config.steps_per_mm[0]; break;
        case 101: cnc_config.steps_per_mm[1]=CNC_Config_ClampFloat(value,MIN_STEPS_PER_MM,MAX_STEPS_PER_MM); applied=cnc_config.steps_per_mm[1]; break;
        case 102: cnc_config.steps_per_mm[2]=CNC_Config_ClampFloat(value,MIN_STEPS_PER_MM,MAX_STEPS_PER_MM); applied=cnc_config.steps_per_mm[2]; break;
        case 103: cnc_config.steps_per_mm[3]=CNC_Config_ClampFloat(value,MIN_STEPS_PER_MM,MAX_STEPS_PER_MM); applied=cnc_config.steps_per_mm[3]; break;

        case 110: cnc_config.max_feedrate[0]=CNC_Config_ClampFloat(value,1,1000000); applied=cnc_config.max_feedrate[0]; break;
        case 111: cnc_config.max_feedrate[1]=CNC_Config_ClampFloat(value,1,1000000); applied=cnc_config.max_feedrate[1]; break;
        case 112: cnc_config.max_feedrate[2]=CNC_Config_ClampFloat(value,1,1000000); applied=cnc_config.max_feedrate[2]; break;
        case 113: cnc_config.max_feedrate[3]=CNC_Config_ClampFloat(value,1,1000000); applied=cnc_config.max_feedrate[3]; break;

#if CNC_USE_ACCELERATION
        case 120: cnc_config.acceleration[0]=CNC_Config_ClampFloat(value,0.1f,1000000); applied=cnc_config.acceleration[0]; break;
        case 121: cnc_config.acceleration[1]=CNC_Config_ClampFloat(value,0.1f,1000000); applied=cnc_config.acceleration[1]; break;
        case 122: cnc_config.acceleration[2]=CNC_Config_ClampFloat(value,0.1f,1000000); applied=cnc_config.acceleration[2]; break;
        case 123: cnc_config.acceleration[3]=CNC_Config_ClampFloat(value,0.1f,1000000); applied=cnc_config.acceleration[3]; break;
#endif
        case 130: cnc_config.max_travel[0]=CNC_Config_ClampFloat(value,0,1000000); applied=cnc_config.max_travel[0]; break;
        case 131: cnc_config.max_travel[1]=CNC_Config_ClampFloat(value,0,1000000); applied=cnc_config.max_travel[1]; break;
        case 132: cnc_config.max_travel[2]=CNC_Config_ClampFloat(value,0,1000000); applied=cnc_config.max_travel[2]; break;
        case 133: cnc_config.max_travel[3]=CNC_Config_ClampFloat(value,0,1000000); applied=cnc_config.max_travel[3]; break;

        default: CNC_UART_PutString("[UNKNOWN PARAM]\r\n"); return;
    }
    CNC_UART_PutString("[SET $"); CNC_UART_PutInt(idx);
    CNC_UART_PutString("="); CNC_UART_PutFloat(applied,3); CNC_UART_PutString("]\r\n");
}

void CNC_Config_SetStepsPerMM(uint8_t axis, float v) {
    if (axis<CNC_NUM_AXES) cnc_config.steps_per_mm[axis]=CNC_Config_ClampFloat(v,MIN_STEPS_PER_MM,MAX_STEPS_PER_MM);
}
void CNC_Config_SetMaxFeedrate(uint8_t axis, float v) {
    if (axis<CNC_NUM_AXES) cnc_config.max_feedrate[axis]=CNC_Config_ClampFloat(v,1,1000000);
}
void CNC_Config_SetMaxTravel(uint8_t axis, float v) {
    if (axis<CNC_NUM_AXES) cnc_config.max_travel[axis]=CNC_Config_ClampFloat(v,0,1000000);
}
void CNC_Config_SetAcceleration(uint8_t axis, float v) {
#if CNC_USE_ACCELERATION
    if (axis<CNC_NUM_AXES) cnc_config.acceleration[axis]=CNC_Config_ClampFloat(v,0.1f,1000000);
#endif
}
void CNC_Config_SetSoftLimit(bool e) { cnc_config.soft_limit_enabled=e; }
void CNC_Config_SetAutoReport(uint32_t ms) {
    if (ms>3600000) ms=3600000;
    cnc_config.auto_report_interval=ms; auto_report_counter=0;
}

void CNC_Config_Show(void)
{
    CNC_UART_PutString("\r\n=== CNC CONFIG v"); CNC_UART_PutString(CNC_VERSION_STRING); CNC_UART_PutString(" ===\r\n");
    const char *an[4]={"X","Y","Z","V"};
    CNC_UART_PutString("\r\nSteps/mm:\r\n");
    for(int i=0;i<4;i++){CNC_UART_PutString("  $10");CNC_UART_PutInt(i);CNC_UART_PutChar(' ');CNC_UART_PutString(an[i]);CNC_UART_PutChar('=');CNC_UART_PutFloat(cnc_config.steps_per_mm[i],2);CNC_UART_PutString("\r\n");}
    CNC_UART_PutString("\r\nMax Rate (mm/min):\r\n");
    for(int i=0;i<4;i++){CNC_UART_PutString("  $11");CNC_UART_PutInt(i);CNC_UART_PutChar(' ');CNC_UART_PutString(an[i]);CNC_UART_PutChar('=');CNC_UART_PutFloat(cnc_config.max_feedrate[i],0);CNC_UART_PutString("\r\n");}
#if CNC_USE_ACCELERATION
    CNC_UART_PutString("\r\nAccel (mm/s2):\r\n");
    for(int i=0;i<4;i++){CNC_UART_PutString("  $12");CNC_UART_PutInt(i);CNC_UART_PutChar(' ');CNC_UART_PutString(an[i]);CNC_UART_PutChar('=');CNC_UART_PutFloat(cnc_config.acceleration[i],1);CNC_UART_PutString("\r\n");}
#endif
    CNC_UART_PutString("\r\nDir Invert: $3="); CNC_UART_PutInt(cnc_config.dir_invert_mask);
    CNC_UART_PutString(" (bit2=Z, bit3=V inverted)\r\n");
    CNC_UART_PutString("Step Invert: $2="); CNC_UART_PutInt(cnc_config.step_invert_mask); CNC_UART_PutString("\r\n");
#if CNC_USE_HOMING
    CNC_UART_PutString("\r\nHoming: $22="); CNC_UART_PutInt(cnc_config.homing_enabled);
    CNC_UART_PutString(" DirMask="); CNC_UART_PutInt(cnc_config.homing_dir_mask);
    CNC_UART_PutString(" AxisMask="); CNC_UART_PutInt(cnc_config.homing_axis_mask); CNC_UART_PutString("\r\n");
#endif
#if CNC_USE_GUI_PROTOCOL
    CNC_UART_PutString("\r\nPresets (mm):\r\n");
    const char pn[4]={'A','B','C','D'};
    for(int i=0;i<GUI_NUM_PRESETS;i++){
        CNC_UART_PutString("  "); CNC_UART_PutChar(pn[i]); CNC_UART_PutString(": ");
        for(int j=0;j<CNC_NUM_AXES;j++){
            CNC_UART_PutChar(CNC_AxisName(j)); CNC_UART_PutChar('=');
            if(gui_proto.presets[i].axis[j]==GUI_SENTINEL_PULSE) CNC_UART_PutString("N/A ");
            else{CNC_UART_PutFloat(gui_proto.presets[i].axis[j]/GUI_PULSE_PER_MM,3);CNC_UART_PutChar(' ');}
        }
        CNC_UART_PutString("\r\n");
    }
#endif
    CNC_UART_PutString("===========================\r\n");
}

/*============================================================================
 *                    G-CODE PARSER
 *============================================================================*/

static bool find_value(const char *line, char letter, float *value) {
    if (!line || !value) return false;
    char upper=letter, lower=letter;
    if (letter>='a'&&letter<='z') upper=letter-32;
    else if (letter>='A'&&letter<='Z') lower=letter+32;
    const char *p = line;
    while (*p) {
        if (*p==upper || *p==lower) {
            bool ok = (p==line)||(*(p-1)==' ')||(*(p-1)=='\t');
            if (ok) {
                const char *ns = p+1;
                while(*ns==' '||*ns=='\t') ns++;
                char fc=*ns;
                if(fc=='-'||fc=='+'||fc=='.'||(fc>='0'&&fc<='9')){
                    char *ep; float v=strtof(ns,&ep);
                    if(ep!=ns){*value=v;return true;}
                }
            }
        }
        p++;
    }
    return false;
}

static bool has_command(const char *line, char letter, int code) {
    float v; return find_value(line,letter,&v) && ((int)v==code);
}

uint8_t CNC_Execute(const char *line)
{
    float value;
    if (!line || line[0]=='\0' || line[0]==';' || line[0]=='(') return 0;

    /* Convert to uppercase for parsing */
    char upper_line[GCODE_LINE_SIZE];
    int li=0;
    while (line[li] && li<GCODE_LINE_SIZE-1) {
        char c=line[li];
        if(c>='a'&&c<='z') c-=32;
        upper_line[li]=c; li++;
    }
    upper_line[li]='\0';
    line = upper_line;

    if (has_command(line,'G',0))  rapid_mode=true;
    if (has_command(line,'G',1))  rapid_mode=false;
    if (has_command(line,'G',20)) cnc_machine.unit=CNC_UNIT_INCH;
    if (has_command(line,'G',21)) cnc_machine.unit=CNC_UNIT_MM;
    if (has_command(line,'G',90)) cnc_machine.coord_mode=CNC_COORD_ABSOLUTE;
    if (has_command(line,'G',91)) cnc_machine.coord_mode=CNC_COORD_INCREMENTAL;

    if (has_command(line,'G',28)) {
#if CNC_USE_HOMING
        CNC_HomeAll();
#else
        for(int i=0;i<CNC_NUM_AXES;i++) cnc_machine.position[i]=0;
        CNC_UART_PutString("[POSITION RESET]\r\n");
#endif
        return 0;
    }

    if (has_command(line,'G',92)) {
        float pos[CNC_NUM_AXES]; CNC_GetPosition(pos);
        if(find_value(line,'X',&value)) pos[0]=value;
        if(find_value(line,'Y',&value)) pos[1]=value;
        if(find_value(line,'Z',&value)) pos[2]=value;
        if(find_value(line,'V',&value)) pos[3]=value;
        CNC_SetPosition(pos); return 0;
    }

    if (has_command(line,'M',17)) CNC_EnableMotors(true);
    if (has_command(line,'M',18)||has_command(line,'M',84)) CNC_EnableMotors(false);
#if CNC_USE_RELAY
    if (has_command(line,'M',3))  CNC_Spindle_On();
    if (has_command(line,'M',5))  CNC_Spindle_Off();
    if (has_command(line,'M',7))  CNC_Coolant_Mist_On();
    if (has_command(line,'M',8))  CNC_Coolant_Flood_On();
    if (has_command(line,'M',9))  CNC_Coolant_Off();
#endif
    if (has_command(line,'M',114)){ CNC_SendStatus(); return 0; }
    if (has_command(line,'M',0)||has_command(line,'M',2)||has_command(line,'M',30)){
        CNC_Stop();
#if CNC_USE_RELAY
        CNC_Spindle_Off(); CNC_Coolant_Off();
#endif
        return 0;
    }

    float target[CNC_NUM_AXES]; CNC_GetPosition(target);
    bool has_motion=false;
    float feedrate=current_feedrate;

    if(find_value(line,'X',&value)){
        target[0]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?value:target[0]+value; has_motion=true;
    }
    if(find_value(line,'Y',&value)){
        target[1]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?value:target[1]+value; has_motion=true;
    }
    if(find_value(line,'Z',&value)){
        target[2]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?value:target[2]+value; has_motion=true;
    }
    if(find_value(line,'V',&value)){
        target[3]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?value:target[3]+value; has_motion=true;
    }
    if(find_value(line,'F',&value)){ feedrate=value; current_feedrate=feedrate; }

    if (has_motion) {
        if (cnc_machine.unit==CNC_UNIT_INCH) {
            for(int i=0;i<CNC_NUM_AXES;i++) target[i]*=25.4f;
            feedrate*=25.4f;
        }
        if (cnc_config.soft_limit_enabled) {
            for(int i=0;i<CNC_NUM_AXES;i++){
                float mn=cnc_config.allow_negative?-cnc_config.max_travel[i]:0;
                if(target[i]<mn||target[i]>cnc_config.max_travel[i]) return 6;
            }
        }
#if CNC_USE_PLANNER
        cnc_machine.gui_action = GUI_ACTION_GCODE_MOVE;
        if (!CNC_Planner_AddBlock(target,feedrate,rapid_mode)) return 9;
#else
        cnc_machine.gui_action = GUI_ACTION_GCODE_MOVE;
        cnc_machine.state = CNC_STATE_RUN;
        CNC_Move(target,feedrate,rapid_mode);
#endif
    }
    return 0;
}

/*============================================================================
 *                    STATUS
 *============================================================================*/

void CNC_SendStatus(void)
{
#if CNC_USE_GUI_PROTOCOL
    CNC_GUI_SendStatus();
#else
    CNC_UART_PutString("<");
    switch(cnc_machine.state){
        case CNC_STATE_IDLE:   CNC_UART_PutString("Idle");  break;
        case CNC_STATE_RUN:    CNC_UART_PutString("Run");   break;
        case CNC_STATE_HOLD:   CNC_UART_PutString("Hold");  break;
        case CNC_STATE_HOMING: CNC_UART_PutString("Home");  break;
        case CNC_STATE_ALARM:  CNC_UART_PutString("Alarm"); break;
        case CNC_STATE_LIMIT_TRIGGERED: CNC_UART_PutString("Limit"); break;
        case CNC_STATE_GOTO:   CNC_UART_PutString("GoTo");  break;
        default: CNC_UART_PutString("?"); break;
    }
    CNC_UART_PutString("|WPos:");
    for(int i=0;i<CNC_NUM_AXES;i++){
        CNC_UART_PutFloat(cnc_machine.position[i],3);
        if(i<CNC_NUM_AXES-1) CNC_UART_PutChar(',');
    }
    CNC_UART_PutString(">\r\n");
#endif
}

void CNC_SendStatusCompact(void)
{
#if CNC_USE_GUI_PROTOCOL
    CNC_GUI_SendStatus();
#else
    CNC_UART_PutChar('<');
    switch(cnc_machine.state){
        case CNC_STATE_IDLE:   CNC_UART_PutChar('I'); break;
        case CNC_STATE_RUN:    CNC_UART_PutChar('R'); break;
        case CNC_STATE_HOLD:   CNC_UART_PutChar('H'); break;
        case CNC_STATE_HOMING: CNC_UART_PutChar('G'); break;
        case CNC_STATE_ALARM:  CNC_UART_PutChar('A'); break;
        case CNC_STATE_LIMIT_TRIGGERED: CNC_UART_PutChar('L'); break;
        case CNC_STATE_GOTO:   CNC_UART_PutChar('T'); break;
        default: CNC_UART_PutChar('?'); break;
    }
    CNC_UART_PutChar('|');
    for(int i=0;i<CNC_NUM_AXES;i++){
        CNC_UART_PutFloat(cnc_machine.position[i],2);
        if(i<CNC_NUM_AXES-1) CNC_UART_PutChar(',');
    }
    CNC_UART_PutString(">\r\n");
#endif
}

/*============================================================================
 *                    CNC_ProcessCommand
 *============================================================================*/

void CNC_ProcessCommand(const char *cmd)
{
#if CNC_USE_GUI_PROTOCOL
    /*--- GUI Protocol commands (case-sensitive per spec) ---*/
    if (strcmp(cmd,"STOP")==0 || strcmp(cmd,"!")==0) {
        CNC_GUI_Handle_STOP(); return;
    }
    if (strncmp(cmd,"GOTO ",5)==0) {
        if (cnc_machine.state!=CNC_STATE_IDLE &&
            cnc_machine.state!=CNC_STATE_HOLD) {
            /* BUSY - silent ignore except STOP */
            return;
        }
        CNC_GUI_Handle_GOTO(cmd+5); return;
    }
    if (strncmp(cmd,"SET ",4)==0) {
        if (cnc_machine.state==CNC_STATE_GOTO ||
            cnc_machine.state==CNC_STATE_RUN) return; /* BUSY */
        CNC_GUI_Handle_SET(cmd+4); return;
    }
    if (strncmp(cmd,"RETURN ",7)==0) {
        CNC_GUI_Handle_RETURN(cmd+7); return;
    }
#endif

    /* GOWDELAYRECT */
    if (strncmp(cmd,"GOWDELAYRECT",12)==0) {
        const char *p=cmd+12; while(*p==' ') p++;
        float sx=0,sy=0,sd=0,ff=0; uint32_t wm=0;
        bool hx=false,hy=false,hd=false,hw=false; float v;
        if(find_value(p,'X',&v)){sx=v;hx=true;}
        if(find_value(p,'Y',&v)){sy=v;hy=true;}
        if(find_value(p,'D',&v)){sd=v;hd=true;}
        if(find_value(p,'W',&v)){wm=(uint32_t)v;hw=true;}
        if(find_value(p,'F',&v)) ff=v;
        if(!hx||!hy){CNC_UART_PutString("[ERROR: X,Y required]\r\n");return;}
        if(!hd){CNC_UART_PutString("[ERROR: D required]\r\n");return;}
        if(!hw){CNC_UART_PutString("[ERROR: W required]\r\n");return;}
        CNC_GoWDelayRect_Start(sx,sy,sd,wm,ff); return;
    }

    /* GOWDELAY */
    if (strncmp(cmd,"GOWDELAY",8)==0) {
        const char *p=cmd+8; while(*p==' ') p++;
        float target[CNC_NUM_AXES]; CNC_GetPosition(target);
        float sd=0,ff=0; uint32_t wm=0;
        bool hm=false,hd=false,hw=false; float v;
        if(find_value(p,'X',&v)){target[0]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?v:target[0]+v;hm=true;}
        if(find_value(p,'Y',&v)){target[1]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?v:target[1]+v;hm=true;}
        if(find_value(p,'Z',&v)){target[2]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?v:target[2]+v;hm=true;}
        if(find_value(p,'V',&v)){target[3]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?v:target[3]+v;hm=true;}
        if(find_value(p,'D',&v)){sd=v;hd=true;}
        if(find_value(p,'W',&v)){wm=(uint32_t)v;hw=true;}
        if(find_value(p,'F',&v)) ff=v;
        if(!hm){CNC_UART_PutString("[ERROR: No axis]\r\n");return;}
        if(!hd){CNC_UART_PutString("[ERROR: D required]\r\n");return;}
        if(!hw){CNC_UART_PutString("[ERROR: W required]\r\n");return;}
        CNC_GoWDelay_Start(target,sd,wm,ff); return;
    }

    /* SET position (legacy) */
    if (strncmp(cmd,"SET ",4)==0 && cmd[4]>='A' && cmd[4]<='Z') {
        char axis=cmd[4]; float v=atof(&cmd[5]);
        uint8_t ax=(axis=='X')?0:(axis=='Y')?1:(axis=='Z')?2:(axis=='V')?3:255;
        if(ax<CNC_NUM_AXES){
            CNC_SetAxisPosition(ax,v);
            CNC_UART_PutString("[SET "); CNC_UART_PutChar(axis);
            CNC_UART_PutChar('='); CNC_UART_PutFloat(v,2); CNC_UART_PutString("]\r\n");
        }
        return;
    }

    /* Config */
    if (cmd[0]=='$' && strchr(cmd,'=')) {
        char *eq=(char*)strchr(cmd,'='); float v=atof(eq+1);
        char param[12]; int ln=(int)(eq-cmd); if(ln>11)ln=11;
        strncpy(param,cmd,ln); param[ln]='\0';
        CNC_Config_Set(param,v); return;
    }
    if (strcmp(cmd,"$$")==0||strcmp(cmd,"CONFIG")==0){ CNC_Config_Show(); return; }

    /* Control */
    if (strcmp(cmd,"STOP")==0||strcmp(cmd,"!")==0) {
        if(cnc_machine.state==CNC_STATE_GOWDELAY) CNC_GoWDelay_Stop();
        else if(cnc_machine.state==CNC_STATE_GOWDELAY_RECT) CNC_GoWDelayRect_Stop();
        else { CNC_Stop(); CNC_UART_PutString("[STOPPED]\r\n"); }
        return;
    }
    if (strcmp(cmd,"RESET")==0) {
        CNC_Stop(); CNC_EnableMotors(false);
#if CNC_USE_RELAY
        CNC_Spindle_Off(); CNC_Coolant_Off();
#endif
        CNC_LoadDefaultConfig();
        for(int i=0;i<CNC_NUM_AXES;i++){cnc_machine.position[i]=0;}
        cnc_machine.state=CNC_STATE_IDLE;
        cnc_machine.gowdelay.phase=GOWDELAY_IDLE;
        cnc_machine.gowdelay_rect.phase=GWDRECT_IDLE;
        current_feedrate=DEFAULT_FEEDRATE; rapid_mode=false;
#if CNC_USE_COMMAND_QUEUE
        CNC_Queue_Clear();
#endif
#if CNC_USE_PLANNER
        CNC_Planner_Clear();
#endif
#if CNC_USE_GUI_PROTOCOL
        cnc_machine.gui_action=GUI_ACTION_IDLE;
        CNC_GUI_Init();
#endif
        CNC_UART_PutString("[RESET OK]\r\n"); return;
    }
    if (strcmp(cmd,"STATUS")==0||strcmp(cmd,"?")==0){ CNC_SendStatus(); return; }
    if (strcmp(cmd,"PAUSE")==0){
        if(cnc_machine.state==CNC_STATE_RUN){CNC_Timer_Stop();cnc_machine.state=CNC_STATE_HOLD;CNC_UART_PutString("[PAUSED]\r\n");}
        return;
    }
    if (strcmp(cmd,"RESUME")==0){
        if(cnc_machine.state==CNC_STATE_HOLD){CNC_Timer_Start();cnc_machine.state=CNC_STATE_RUN;CNC_UART_PutString("[RESUMED]\r\n");}
        return;
    }
#if CNC_USE_HOMING
    if (strcmp(cmd,"HOME")==0){ CNC_HomeAll(); return; }
    if (strcmp(cmd,"HOMEX")==0){ CNC_HomeAxis(0); return; }
    if (strcmp(cmd,"HOMEY")==0){ CNC_HomeAxis(1); return; }
    if (strcmp(cmd,"HOMEZ")==0){ CNC_HomeAxis(2); return; }
    if (strcmp(cmd,"HOMEV")==0){ CNC_HomeAxis(3); return; }
#endif
    if (strcmp(cmd,"ENABLE")==0){ CNC_EnableMotors(true);  CNC_UART_PutString("[MOTORS ON]\r\n");  return; }
    if (strcmp(cmd,"DISABLE")==0){ CNC_EnableMotors(false); CNC_UART_PutString("[MOTORS OFF]\r\n"); return; }
    if (strcmp(cmd,"LIMITON")==0){ cnc_config.soft_limit_enabled=true;  CNC_UART_PutString("[SOFT LIMIT ON]\r\n");  return; }
    if (strcmp(cmd,"LIMITOFF")==0){ cnc_config.soft_limit_enabled=false; CNC_UART_PutString("[SOFT LIMIT OFF]\r\n"); return; }
    if (strcmp(cmd,"CLEAR")==0){
#if CNC_USE_LIMIT_SWITCH
        CNC_Limit_ClearAlarm();
#else
        cnc_machine.state=CNC_STATE_IDLE; CNC_UART_PutString("[ALARM CLEARED]\r\n");
#endif
        return;
    }
#if CNC_USE_RELAY
    if (strcmp(cmd,"SPINON")==0){ CNC_Spindle_On();      CNC_UART_PutString("[SPINDLE ON]\r\n");  return; }
    if (strcmp(cmd,"SPINOFF")==0){ CNC_Spindle_Off();     CNC_UART_PutString("[SPINDLE OFF]\r\n"); return; }
    if (strcmp(cmd,"MISTON")==0){ CNC_Coolant_Mist_On(); CNC_UART_PutString("[MIST ON]\r\n");     return; }
    if (strcmp(cmd,"MISTOFF")==0){ CNC_Coolant_Mist_Off();CNC_UART_PutString("[MIST OFF]\r\n");    return; }
    if (strcmp(cmd,"FLOODON")==0){ CNC_Coolant_Flood_On();CNC_UART_PutString("[FLOOD ON]\r\n");    return; }
    if (strcmp(cmd,"FLOODOFF")==0){CNC_Coolant_Flood_Off();CNC_UART_PutString("[FLOOD OFF]\r\n");  return; }
    if (strcmp(cmd,"COOLOFF")==0){ CNC_Coolant_Off();     CNC_UART_PutString("[COOLANT OFF]\r\n"); return; }
#endif
#if CNC_USE_COMMAND_QUEUE
    if (strcmp(cmd,"QUEUE")==0){ CNC_UART_PutString("[QUEUE: "); CNC_UART_PutInt(CNC_Queue_Count()); CNC_UART_PutString("/"); CNC_UART_PutInt(CMD_QUEUE_SIZE); CNC_UART_PutString("]\r\n"); return; }
    if (strcmp(cmd,"CLEARQUEUE")==0){ CNC_Queue_Clear(); CNC_UART_PutString("[QUEUE CLEARED]\r\n"); return; }
#endif
#if CNC_USE_PLANNER
    if (strcmp(cmd,"PLANNER")==0){ CNC_UART_PutString("[PLANNER: "); CNC_UART_PutInt(CNC_Planner_BlocksCount()); CNC_UART_PutString("/"); CNC_UART_PutInt(PLANNER_BUFFER_SIZE); CNC_UART_PutString("]\r\n"); return; }
    if (strcmp(cmd,"CLEARPLANNER")==0){ CNC_Planner_Clear(); CNC_UART_PutString("[PLANNER CLEARED]\r\n"); return; }
#endif
    if (strncmp(cmd,"REPORT ",7)==0){ uint32_t iv=atoi(&cmd[7]); CNC_Config_SetAutoReport(iv); CNC_UART_PutString("[REPORT="); CNC_UART_PutInt(iv); CNC_UART_PutString("ms]\r\n"); return; }
    if (strcmp(cmd,"REPORT")==0){ CNC_Config_SetAutoReport(0); CNC_UART_PutString("[REPORT OFF]\r\n"); return; }
#if CNC_USE_LIMIT_SWITCH
    if (strcmp(cmd,"LIMITS")==0){
        CNC_UART_PutString("[LIMITS: ");
        for(int i=0;i<CNC_NUM_AXES;i++){ CNC_UART_PutChar(CNC_AxisName(i)); CNC_UART_PutChar('='); CNC_UART_PutInt(CNC_Limit_IsTriggered(i)); CNC_UART_PutChar(' '); }
        CNC_UART_PutString("]\r\n"); return;
    }
#endif
    if (strcmp(cmd,"HELP")==0){
        CNC_UART_PutString("\r\n=== CNC v"); CNC_UART_PutString(CNC_VERSION_STRING); CNC_UART_PutString(" HELP ===\r\n");
#if CNC_USE_GUI_PROTOCOL
        CNC_UART_PutString("GUI: STOP GOTO A/B/C/D SET A X.. RETURN A\r\n");
#endif
        CNC_UART_PutString("G-code: G0 G1 G28 G90 G91 G92 X Y Z V F\r\n");
        CNC_UART_PutString("GOWDELAY X Y Z V D W [F]\r\n");
        CNC_UART_PutString("GOWDELAYRECT X Y D W [F]\r\n");
        CNC_UART_PutString("STOP PAUSE RESUME RESET STATUS\r\n");
        CNC_UART_PutString("HOME HOMEX HOMEY HOMEZ HOMEV\r\n");
        CNC_UART_PutString("ENABLE DISABLE LIMITON LIMITOFF CLEAR\r\n");
        CNC_UART_PutString("SPINON SPINOFF MISTON MISTOFF FLOODON FLOODOFF\r\n");
        CNC_UART_PutString("$$ $100=1000 REPORT 300\r\n");
        CNC_UART_PutString("Dir invert: $3=12 (Z,V)\r\n");
        CNC_UART_PutString("===========================\r\n"); return;
    }

    CNC_UART_PutString("[UNKNOWN: "); CNC_UART_PutString(cmd); CNC_UART_PutString("]\r\n");
}

/*============================================================================
 *                    CNC_Init
 *============================================================================*/

void CNC_Init(void)
{
    CNC_SystemClock_Config();
    CNC_LoadDefaultConfig();
    CNC_GPIO_Init();
    CNC_Timer_Init();
    CNC_UART_Init();
#if CNC_USE_DMA_UART
    CNC_UART_DMA_Init();
#endif
#if CNC_USE_LIMIT_SWITCH
    CNC_Limit_Init();
#endif
#if CNC_USE_RELAY
    CNC_Relay_Init();
#endif
#if CNC_USE_COMMAND_QUEUE
    CNC_Queue_Init();
#endif
#if CNC_USE_PLANNER
    CNC_Planner_Init();
#endif
#if CNC_USE_GUI_PROTOCOL
    CNC_GUI_Init();
#endif

    /* Machine state init */
    cnc_machine.state      = CNC_STATE_IDLE;
    cnc_machine.coord_mode = CNC_COORD_ABSOLUTE;
    cnc_machine.unit       = CNC_UNIT_MM;
    cnc_machine.is_moving  = false;
    cnc_machine.motors_enabled = false;
    cnc_machine.feedrate   = DEFAULT_FEEDRATE;
    cnc_machine.spindle_on = cnc_machine.coolant_mist_on = cnc_machine.coolant_flood_on = false;

    for (int i=0;i<CNC_NUM_AXES;i++) {
        cnc_machine.position[i] = cnc_machine.target[i] = 0;
        cnc_machine.step_count[i] = cnc_machine.direction[i] = 0;
        cnc_machine.limit_triggered[i] = false;
        cnc_machine.logical_delta[i] = 0;
        cnc_machine.logical_steps_total[i] = 0;
#if CNC_USE_HOMING
        cnc_machine.homed[i] = false;
#endif
    }

#if CNC_USE_HOMING
    cnc_machine.homing_phase = HOMING_IDLE;
    cnc_machine.homing_axis  = 0;
    cnc_machine.homing_cycle_step   = 0;
    cnc_machine.homing_axes_pending = 0;
#endif

#if CNC_USE_ACCELERATION
    AccelState_t *a = &cnc_machine.accel;
    a->phase=ACCEL_PHASE_ACCEL; a->current_speed=a->target_speed=0;
    a->entry_speed=a->exit_speed=a->accel_rate=0;
    a->accel_steps=a->cruise_steps=a->decel_steps=a->decel_start=0;
    a->mm_per_step=0; a->spm_dominant=1; a->inv_spm_dominant=1;
    a->last_update_tick=0; a->needs_recalc=false;
    a->precomp_high_ticks=a->precomp_low_ticks=a->precomp_period=0;
#endif

#if CNC_USE_GUI_PROTOCOL
    cnc_machine.gui_action = GUI_ACTION_IDLE;
#endif

    cnc_machine.gowdelay.phase      = GOWDELAY_IDLE;
    cnc_machine.gowdelay_rect.phase = GWDRECT_IDLE;

    bresenham_total = step_events_remaining = 0;

    for (volatile int i=0;i<100000;i++);

    CNC_UART_PutString("\r\n================================\r\n");
    CNC_UART_PutString("  CNC Controller v"); CNC_UART_PutString(CNC_VERSION_STRING); CNC_UART_PutString("\r\n");
    CNC_UART_PutString("  4-Axis: X(PA1) Y(PA0) Z(PA6) V(PB0)\r\n");
    CNC_UART_PutString("  Dir: PB5 PB6 PB7(inv) PB8(inv)\r\n");
#if CNC_USE_GUI_PROTOCOL
    CNC_UART_PutString("  + GUI Protocol (GOTO/SET/RETURN)\r\n");
#endif
    CNC_UART_PutString("================================\r\n");
    CNC_UART_PutString("Type HELP for commands\r\n\r\n");
}

/*============================================================================
 *                    CNC_Process
 *============================================================================*/

void CNC_Process(void)
{
    static char line[GCODE_LINE_SIZE];

    /* Auto report */
#if CNC_AUTO_REPORT_ENABLE
    if (cnc_auto_report_flag) {
        cnc_auto_report_flag = false;
#if CNC_USE_GUI_PROTOCOL
        CNC_GUI_SendStatus();
#else
        CNC_SendStatusCompact();
#endif
    }
#endif

    /* Acceleration update */
#if CNC_USE_ACCELERATION
    CNC_Accel_Update();
#endif

    /* Homing */
#if CNC_USE_HOMING
    CNC_Homing_Process();
#endif

    /* GOWDELAY */
    CNC_GoWDelay_Process();
    CNC_GoWDelayRect_Process();

    /* Limit check */
#if CNC_USE_LIMIT_SWITCH
    if (cnc_config.hard_limit_enabled && cnc_machine.is_moving &&
        (cnc_machine.state==CNC_STATE_RUN  || cnc_machine.state==CNC_STATE_GOTO ||
         cnc_machine.state==CNC_STATE_GOWDELAY || cnc_machine.state==CNC_STATE_GOWDELAY_RECT)) {
        CNC_Limit_CheckAll();
    }
#endif

    /* Send OK */
    if (cnc_send_ok_flag) {
        cnc_send_ok_flag = false;
        if (cnc_machine.state != CNC_STATE_GOWDELAY &&
            cnc_machine.state != CNC_STATE_GOWDELAY_RECT &&
            cnc_machine.state != CNC_STATE_GOTO)
            CNC_UART_SendOK();
    }

    /* Queue → Planner */
#if CNC_USE_COMMAND_QUEUE && CNC_USE_PLANNER
    CNC_ProcessQueuedStreamingCommands();
#endif

    /* Planner block execution */
#if CNC_USE_PLANNER
    if (!cnc_machine.is_moving && cnc_machine.state==CNC_STATE_IDLE &&
        !CNC_Planner_IsBufferEmpty()) {

        PlannerBlock_t *block = CNC_Planner_GetCurrentBlock();
        if (block) {
            float bt[CNC_NUM_AXES], bd=block->distance, bns=block->nominal_speed;
            float ba=block->acceleration;
            float bes=sqrtf(block->entry_speed_sqr), bxs=sqrtf(block->exit_speed_sqr);
            for(int i=0;i<CNC_NUM_AXES;i++) bt[i]=block->target[i];
            CNC_Planner_DiscardCurrentBlock();

            cnc_machine.state = CNC_STATE_RUN;
            float delta[CNC_NUM_AXES];
            int64_t new_bt2=0; int dom2=0;

            for(int i=0;i<CNC_NUM_AXES;i++){
                delta[i]=bt[i]-cnc_machine.position[i];
                cnc_machine.target[i]=bt[i];
                cnc_machine.logical_delta[i]=delta[i];
                cnc_machine.logical_steps_total[i]=(int32_t)fabsf(delta[i]*CNC_SafeStepsPerMM(i));
                float spm=CNC_SafeStepsPerMM(i);
                int64_t as2=(int64_t)roundf(fabsf(delta[i]*spm));
                if(as2>MAX_STEPS_PER_MOVE){CNC_UART_PutString("[ERROR: too long]\r\n");cnc_machine.state=CNC_STATE_IDLE;return;}
                bresenham_steps[i]=as2;
                cnc_machine.step_count[i]=(int32_t)as2;
                cnc_machine.steps_total[i]=(int32_t)as2;
                if(as2>new_bt2){new_bt2=as2;dom2=i;}
                if(delta[i]>0.0001f) CNC_SetDirection(i,1);
                else if(delta[i]<-0.0001f) CNC_SetDirection(i,-1);
                else cnc_machine.direction[i]=0;
            }

            if (new_bt2 > 0) {
                for(int i=0;i<CNC_NUM_AXES;i++) bresenham_counter[i]=new_bt2/2;
                __disable_irq(); bresenham_total=new_bt2; step_events_remaining=new_bt2; __enable_irq();
                if(!cnc_machine.motors_enabled) CNC_EnableMotors(true);
                cnc_machine.dominant_axis=dom2; cnc_machine.move_distance=bd;
#if CNC_USE_ACCELERATION
                CNC_Accel_PlanWithEntryExit(bd, bns*60.0f, ba, bes, bxs);
#else
                float sdom=CNC_SafeStepsPerMM(dom2);
                uint32_t sf=(uint32_t)(bns*sdom);
                if(sf<10) sf=10; if(sf>50000) sf=50000;
                CNC_Timer_SetPeriod(1000000UL/sf);
#endif
                cnc_machine.is_moving=true;
                CNC_Timer_Start();
            } else {
                cnc_machine.state=CNC_STATE_IDLE;
            }
        }
    }
#endif

    /* Receive UART line */
    if (CNC_UART_GetLine(line, sizeof(line))) {

        /* Emergency STOP - always */
        if (strcmp(line,"STOP")==0 || strcmp(line,"!")==0) {
#if CNC_USE_GUI_PROTOCOL
            CNC_GUI_Handle_STOP();
#else
            if(cnc_machine.state==CNC_STATE_GOWDELAY) CNC_GoWDelay_Stop();
            else if(cnc_machine.state==CNC_STATE_GOWDELAY_RECT) CNC_GoWDelayRect_Stop();
            else { CNC_Stop(); CNC_UART_PutString("[EMERGENCY STOP]\r\n"); }
#endif
            return;
        }

        if (strcmp(line,"STATUS")==0||strcmp(line,"?")==0){ CNC_SendStatus(); return; }
        if (strcmp(line,"PAUSE")==0){
            if(cnc_machine.state==CNC_STATE_RUN){CNC_Timer_Stop();cnc_machine.state=CNC_STATE_HOLD;CNC_UART_PutString("[PAUSED]\r\n");}
            return;
        }

#if CNC_USE_GUI_PROTOCOL
        /*--- GUI BUSY check: only STOP (handled above) passes when busy ---*/
        bool gui_busy = (cnc_machine.state==CNC_STATE_GOTO ||
                         cnc_machine.state==CNC_STATE_RUN  ||
                         cnc_machine.state==CNC_STATE_GOWDELAY ||
                         cnc_machine.state==CNC_STATE_GOWDELAY_RECT ||
                         cnc_machine.state==CNC_STATE_HOMING);

        if (gui_busy) {
            /* GOTO - silent ignore per spec */
            return;
        }

        /* GUI commands: GOTO, SET, RETURN (case-sensitive) */
        if (strncmp(line,"GOTO ",5)==0)   { CNC_GUI_Handle_GOTO(line+5); return; }
        if (strncmp(line,"SET ",4)==0)    { CNC_GUI_Handle_SET(line+4);  return; }
        if (strncmp(line,"RETURN ",7)==0) { CNC_GUI_Handle_RETURN(line+7); return; }
#endif

        /* Planner streaming */
#if CNC_USE_PLANNER
        if (cnc_machine.state==CNC_STATE_IDLE || cnc_machine.state==CNC_STATE_HOLD) {
            if (CNC_HandlePlannerStreamingInput(line)) return;
        }
#endif

        /* Normal command processing */
        if (cnc_machine.state==CNC_STATE_IDLE   ||
            cnc_machine.state==CNC_STATE_HOLD    ||
            cnc_machine.state==CNC_STATE_ALARM   ||
            cnc_machine.state==CNC_STATE_LIMIT_TRIGGERED) {

            /* Custom commands */
            if (strncmp(line,"GOWDELAYRECT",12)==0 ||
                strncmp(line,"GOWDELAY",8)==0       ||
                strncmp(line,"SET ",4)==0            ||
                strncmp(line,"REPORT ",7)==0         ||
                line[0]=='$' ||
                strcmp(line,"RESET")==0   || strcmp(line,"RESUME")==0 ||
                strcmp(line,"HOME")==0    || strcmp(line,"HOMEX")==0  ||
                strcmp(line,"HOMEY")==0   || strcmp(line,"HOMEZ")==0  ||
                strcmp(line,"HOMEV")==0   ||
                strcmp(line,"ENABLE")==0  || strcmp(line,"DISABLE")==0 ||
                strcmp(line,"LIMITON")==0 || strcmp(line,"LIMITOFF")==0||
                strcmp(line,"CLEAR")==0   || strcmp(line,"REPORT")==0  ||
                strcmp(line,"LIMITS")==0  || strcmp(line,"CONFIG")==0  ||
                strcmp(line,"HELP")==0    || strcmp(line,"SPINON")==0  ||
                strcmp(line,"SPINOFF")==0 || strcmp(line,"MISTON")==0  ||
                strcmp(line,"MISTOFF")==0 || strcmp(line,"FLOODON")==0 ||
                strcmp(line,"FLOODOFF")==0|| strcmp(line,"COOLOFF")==0 ||
                strcmp(line,"QUEUE")==0   || strcmp(line,"CLEARQUEUE")==0 ||
                strcmp(line,"PLANNER")==0 || strcmp(line,"CLEARPLANNER")==0) {
                CNC_ProcessCommand(line);
            } else {
                /* G-code */
                if (cnc_machine.state==CNC_STATE_ALARM ||
                    cnc_machine.state==CNC_STATE_LIMIT_TRIGGERED) {
                    CNC_UART_PutString("[ALARM - USE CLEAR]\r\n");
                } else {
                    uint8_t err = CNC_Execute(line);
                    if (err) CNC_UART_SendError(err);
                    else if (!cnc_machine.is_moving) {
#if CNC_USE_PLANNER
                        if (CNC_Planner_IsBufferEmpty()) CNC_UART_SendOK();
#else
                        CNC_UART_SendOK();
#endif
                    }
                }
            }
        }
    }
}

/* End of grbl_AxisArm.c */

