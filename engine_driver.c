#include "engine_driver.h"
#include <stm32_ll_gpio.h>
#include <stm32_ll_tim.h>
#include <math.h>
#include <stdio.h>

int engine_driver_init(struct engine_config *engine) {
    int ret;

    // Check device readiness for both PWM channels
    if (!pwm_is_ready_dt(&engine->pwm_fwd) || !pwm_is_ready_dt(&engine->pwm_rev)) {
        return -ENODEV;
    }

    // Optional enable pin check and initialization (PA4 / R_EN & L_EN)
    if (engine->enable.port != NULL) {
        if (!gpio_is_ready_dt(&engine->enable)) {
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&engine->enable, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) return ret;
    }

    // Initialize both PWM channels via Zephyr API to enable timer counter & outputs
    ret = pwm_set_pulse_dt(&engine->pwm_fwd, 0);
    if (ret < 0) return ret;
    ret = pwm_set_pulse_dt(&engine->pwm_rev, 0);
    if (ret < 0) return ret;

    // Resolve period in timer clock cycles once using pwm_fwd channel
    uint64_t cycles_per_sec;
    ret = pwm_get_cycles_per_sec(engine->pwm_fwd.dev, engine->pwm_fwd.channel, &cycles_per_sec);
    if (ret < 0) return ret;

    // Convert period in nanoseconds to timer clock cycles
    engine->period_cycles = (uint32_t)((engine->pwm_fwd.period * cycles_per_sec) / NSEC_PER_SEC);
    engine->last_direction = 0;

    // Report configuration and check hardware resolution
    uint32_t freq_hz = (uint32_t)(NSEC_PER_SEC / engine->pwm_fwd.period);
    printf("[Engine Driver] Dual PWM initialized at %u Hz.\n", freq_hz);
    printf("[Engine Driver] Hardware Resolution: %u steps.\n", engine->period_cycles);
    
    if (engine->period_cycles < 1024) {
        printf("[Engine Driver] WARNING: PWM frequency is high or timer clock is low.\n");
        printf("[Engine Driver]          Resolution is only %u steps (below recommended 1024).\n",
               engine->period_cycles);
    }

#ifdef CONFIG_ENGINE_THREAD_SAFE
    k_mutex_init(&engine->mutex);
#endif

    // Turn enable line HIGH to enable H-bridge driver outputs (active high)
    if (engine->enable.port != NULL) {
        ret = gpio_pin_set_dt(&engine->enable, 1);
        if (ret < 0) return ret;
    }

    // Initial state: Electronic Brake (Both PWM channels = 0%, Enable = HIGH)
    engine_driver_set_speed(engine, 0.0f);

    return 0;
}

void engine_driver_set_speed(struct engine_config *engine, float command) {
#ifdef CONFIG_ENGINE_THREAD_SAFE
    k_mutex_lock(&engine->mutex, K_FOREVER);
#endif

    // Clamp command to [-100.0, 100.0]
    if (command > 100.0f)  command = 100.0f;
    if (command < -100.0f) command = -100.0f;

    int direction = 0;
    float duty_percent = 0.0f;

    if (command > 0.0f) {
        direction = 1;
        duty_percent = command;
    } else if (command < 0.0f) {
        direction = -1;
        duty_percent = -command;
    }

    // Software Dead-time on direction change to prevent shoot-through
    if (direction != engine->last_direction && engine->last_direction != 0) {
        // Safe transition: Disable both outputs (PWM = 0)
        LL_TIM_OC_SetCompareCH1(TIM2, 0);
        LL_TIM_OC_SetCompareCH2(TIM2, 0);
        
        // Delay of ~50 us (144 MHz core clock, ~4 cycles per loop iteration)
        for (volatile int i = 0; i < 1800; i++) {
            __asm__ volatile("nop");
        }
    }

    // Map duty percentage directly to hardware timer clock cycles (Option 1)
    uint32_t pulse_cycles = (uint32_t)((duty_percent * (float)engine->period_cycles) / 100.0f);

    if (direction == 1) {
        // Forward: RPWM (CH1) active, LPWM (CH2) = 0
        LL_TIM_OC_SetCompareCH2(TIM2, 0);
        LL_TIM_OC_SetCompareCH1(TIM2, pulse_cycles);
    } 
    else if (direction == -1) {
        // Reverse: LPWM (CH2) active, RPWM (CH1) = 0
        LL_TIM_OC_SetCompareCH1(TIM2, 0);
        LL_TIM_OC_SetCompareCH2(TIM2, pulse_cycles);
    } 
    else {
        // Active Slow Decay Brake: RPWM (CH1) = 0, LPWM (CH2) = 0 (and Enable is HIGH)
        LL_TIM_OC_SetCompareCH1(TIM2, 0);
        LL_TIM_OC_SetCompareCH2(TIM2, 0);
    }

    engine->last_direction = direction;

#ifdef CONFIG_ENGINE_THREAD_SAFE
    k_mutex_unlock(&engine->mutex);
#endif
}
