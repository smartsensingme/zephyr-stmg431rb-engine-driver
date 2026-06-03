#ifndef ENGINE_DRIVER_H_
#define ENGINE_DRIVER_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

struct engine_config {
    /** Zephyr PWM specification for Forward direction (RPWM, TIM2 CH1). */
    struct pwm_dt_spec pwm_fwd;
    /** Zephyr PWM specification for Reverse direction (LPWM, TIM2 CH2). */
    struct pwm_dt_spec pwm_rev;
    /** GPIO specification for the H-Bridge Enable control pin (R_EN/L_EN). */
    struct gpio_dt_spec enable;
    /** Resolved period of the PWM in timer clock cycles, cached once at initialization. */
    uint32_t period_cycles;
    /** Tracks the last applied direction state (1 for Fwd, -1 for Rev, 0 for Brake) to trigger dead-time. */
    int last_direction;
#ifdef CONFIG_ENGINE_THREAD_SAFE
    /** Mutex to synchronize speed updates across multiple threads. */
    struct k_mutex mutex;
#endif
};

/**
 * @brief Initializes the engine driver pins and resolves cycles.
 */
int engine_driver_init(struct engine_config *engine);

/**
 * @brief Sets the speed / duty cycle of the engine.
 * @param engine Pointer to the config structure.
 * @param command Speed from -100.0f to 100.0f.
 */
void engine_driver_set_speed(struct engine_config *engine, float command);

#endif /* ENGINE_DRIVER_H_ */
