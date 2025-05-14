#include "sr04.h"

// Example limit for ignoring absurd results
#define DISTANCE_LIMIT 4000

void sr04_init(sr04_t *sr04_struct)
{
    // Make sure trigger pin is LOW at start
    HAL_GPIO_WritePin(sr04_struct->trig_port, sr04_struct->trig_pin, GPIO_PIN_RESET);

    // Configure input capture edge to rising initially
    __HAL_TIM_SET_CAPTUREPOLARITY(sr04_struct->echo_htim,
                                  sr04_struct->echo_channel,
                                  TIM_INPUTCHANNELPOLARITY_RISING);

    sr04_struct->capture_flag = 0;
    sr04_struct->tim_update_count = 0;

    // Start capturing + timer interrupts
    HAL_TIM_IC_Start_IT(sr04_struct->echo_htim, sr04_struct->echo_channel);
    HAL_TIM_Base_Start_IT(sr04_struct->echo_htim);
}

void sr04_trigger(sr04_t *sr04_struct)
{
    // Send a ~10us pulse (or more)
    HAL_GPIO_WritePin(sr04_struct->trig_port, sr04_struct->trig_pin, GPIO_PIN_SET);
    HAL_Delay(1); // in ms; for a real 10us pulse, you'd do a microsecond delay
    HAL_GPIO_WritePin(sr04_struct->trig_port, sr04_struct->trig_pin, GPIO_PIN_RESET);
}

void sr04_read_distance(sr04_t *sr04_struct)
{
    // Called from TIM capture callback:
    switch (sr04_struct->capture_flag)
    {
    case 0:
        // Rising edge just arrived
        sr04_struct->start_counter = __HAL_TIM_GET_COUNTER(sr04_struct->echo_htim);
        sr04_struct->tim_update_count = 0;
        sr04_struct->capture_flag = 1;

        // Next edge will be falling
        __HAL_TIM_SET_CAPTUREPOLARITY(sr04_struct->echo_htim,
                                      sr04_struct->echo_channel,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
        break;

    case 1:
        // Falling edge
        sr04_struct->end_counter =
              __HAL_TIM_GET_COUNTER(sr04_struct->echo_htim)
            + sr04_struct->tim_update_count * sr04_struct->echo_htim->Init.Period;

        sr04_struct->capture_flag = 0;

        // Calculate distance in mm (example formula)
        // (Because timer increments in microseconds if Prescaler=63 at 64MHz CPU)
        uint32_t elapsed_us = sr04_struct->end_counter - sr04_struct->start_counter;
        // speed of sound ~340 m/s = 0.34 mm/us, factor of /2 for round-trip
        uint32_t dist_mm = (elapsed_us * 34) / 100; // ~0.34 mm/us
        // but this also depends on your exact clock / prescaler
        // Adjust to match your actual setup

        // Check limit
        if (dist_mm > DISTANCE_LIMIT)
            dist_mm = sr04_struct->last_distance;

        sr04_struct->distance = dist_mm;
        sr04_struct->last_distance = dist_mm;

        // Return to capturing on rising edge again
        __HAL_TIM_SET_CAPTUREPOLARITY(sr04_struct->echo_htim,
                                      sr04_struct->echo_channel,
                                      TIM_INPUTCHANNELPOLARITY_RISING);
        break;
    }
}
