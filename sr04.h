#ifndef SR04_SR04_H
#define SR04_SR04_H

#include "stm32f3xx_hal.h"

typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    TIM_HandleTypeDef *echo_htim;
    uint16_t      echo_channel;
    uint8_t       capture_flag;
    uint32_t      start_counter;
    uint32_t      end_counter;
    uint32_t      distance;       // Distance in mm (for example)
    uint32_t      last_distance;
    uint16_t      tim_update_count;
} sr04_t;

void sr04_init(sr04_t *sr04_struct);
void sr04_trigger(sr04_t *sr04_struct);
void sr04_read_distance(sr04_t *sr04_struct);

#endif // SR04_SR04_H
