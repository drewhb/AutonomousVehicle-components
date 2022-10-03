#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define DRIVE_MIN_PULSEWIDTH 900 //Minimum pulse width in microsecond
#define DRIVE_MAX_PULSEWIDTH 1700 //Maximum pulse width in microsecond
#define DRIVE_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate
#define STEERING_MIN_PULSEWIDTH 700 //Minimum pulse width in microsecond
#define STEERING_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
#define STEERING_MAX_DEGREE 100 //Maximum angle in degree upto which servo can rotate


static void mcpwm_example_gpio_initialize(void)
{
    gpio_pad_select_gpio(25);
    gpio_set_level(25,1);
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);    //18
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 26);    //26
}


static uint32_t drive_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (DRIVE_MIN_PULSEWIDTH + (((DRIVE_MAX_PULSEWIDTH - DRIVE_MIN_PULSEWIDTH) * (degree_of_rotation)) / (DRIVE_MAX_DEGREE)));
    return cal_pulsewidth;
}


static uint32_t steering_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (STEERING_MIN_PULSEWIDTH + (((STEERING_MAX_PULSEWIDTH - STEERING_MIN_PULSEWIDTH) * (degree_of_rotation)) / (STEERING_MAX_DEGREE)));
    return cal_pulsewidth;
}

void pwm_init() {
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}



void calibrateESC() {
    printf("Turn on in 3 seconds\n");
    // vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
    // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds - backwards
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds - forwards
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler (3s)
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
    vTaskDelay(3100 / portTICK_PERIOD_MS); // Do for at least 3s, and leave in neutral state
}



void drive_control(void *arg)
{
     uint32_t angle, count;
     printf("Driving!\n");
    // for (count = 1400; count > 1100; count -= 5) {
    //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
    //     vTaskDelay(100/portTICK_RATE_MS);
    // }

    // for (count = 1200; count < 1600; count += 5) {
    //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
    //     vTaskDelay(100/portTICK_RATE_MS);
    // }

    // for (count = 1600; count >= 1400; count -= 5) {
    //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
    //     vTaskDelay(100/portTICK_RATE_MS);
    // }
    printf("Forward...\n");
    for (count = 1400; count < 1500; count += 5) {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        vTaskDelay(100/portTICK_RATE_MS);
    }
    vTaskDelay(3000/portTICK_RATE_MS);
    printf("Stopping...\n");
    for (count = 1500; count > 1400; count -= 5) {      //stop
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        vTaskDelay(100/portTICK_RATE_MS);
    }
    vTaskDelay(3000/portTICK_RATE_MS);

    printf("Backward calibrate...\n");


    for (count = 1400; count > 700; count -= 5) {      //forward
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        vTaskDelay(25/portTICK_RATE_MS);
    }

    printf("back to center...\n");
    for (count = 700; count < 1400; count += 5) {      //stop
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        vTaskDelay(25/portTICK_RATE_MS);
    }

    printf("Backward...\n");

    for (count = 1400; count > 1300; count -= 5) {      //forward
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        vTaskDelay(100/portTICK_RATE_MS);
    }
    vTaskDelay(3000/portTICK_RATE_MS);
    printf("stopping...\n");
    for (count = 1300; count < 1400; count += 5) {      //stop
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        vTaskDelay(100/portTICK_RATE_MS);
    }

    //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200);
    vTaskDelay(1000/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);
    vTaskDelay(100/portTICK_RATE_MS);



    vTaskDelete(NULL);
}


void steering_control(void *arg)
{
    uint32_t angle, count;
    vTaskDelay(25000/portTICK_RATE_MS);
      while (1) {
          printf("Steering!\n");
          for (count = 50; count < STEERING_MAX_DEGREE; count++) {
              //count = 90;
              //printf("Angle of rotation: %d\n", count);
              angle = steering_per_degree_init(count);
              //printf("pulse width: %dus\n", angle);
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
              vTaskDelay(25/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
          }
          for (count = STEERING_MAX_DEGREE; count > 0; count--) {
              //count = 90;
              //printf("Angle of rotation: %d\n", count);
              angle = steering_per_degree_init(count);
              //printf("pulse width: %dus\n", angle);
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
              vTaskDelay(25/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
          }

          for (count = 0; count <50; count++) {
              //count = 90;
              //printf("Angle of rotation: %d\n", count);
              angle = steering_per_degree_init(count);
              //printf("pulse width: %dus\n", angle);
              mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
              vTaskDelay(25/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
          }
          vTaskDelay(3000/portTICK_RATE_MS);

      }
}



void app_main(void)
{

    pwm_init();
    calibrateESC();

    printf("Testing servo motor.......\n");
    xTaskCreate(steering_control, "steering_control", 4096, NULL, 5, NULL);
    xTaskCreate(drive_control, "drive_control", 4096, NULL, 5, NULL);


}
