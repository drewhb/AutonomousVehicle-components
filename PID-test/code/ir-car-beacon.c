
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/i2c.h"

// LED Output pins definitions
#define BLUEPIN   14
#define GREENPIN  32
#define REDPIN    15

// I2C definitions for LIDAR


// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// LIDARLite_v4LED slave address
#define SLAVE_ADDR                         0x62 // slave address


// // Timer definitions
// #define TIMER_DIVIDER         16    //  Hardware timer clock divider
// #define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
// #define TIMER_INTERVAL_2_SEC  (2)
// #define TIMER_INTERVAL_10_SEC (10)
// #define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

int dt = 100;
float integral;
float derivative;

int setpoint = 20;
int16_t distance;
// Flag for dt
int dt_complete = 0;


// GPIO init for LEDs
static void led_init() {
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
}


static void i2c_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    conf.clk_flags = 0;
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {printf("- initialized: yes\n");}

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}


int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0) {printf("- No I2C devices found!" "\n");}
}


// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
    // YOUR CODE HERE
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read register
uint8_t readRegister(uint8_t reg) {
    // YOUR CODE HERE
    int ret;
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
    // YOUR CODE HERE
    int ret;
    uint8_t data, data2;
    uint16_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, ACK_VAL);
    i2c_master_read_byte(cmd, &data2, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    result = (data2 << 8) | data;
    // printf("\nread16 Data: %d\n", result);
    return result;
}


static void LIDAR(){
    printf("\n>> Polling LIDARLite_v4LED!\n");
    // variables

    uint8_t busyflag = 1;

    while (1) {
        writeRegister(0x00, 0x04);  // start the read/write - modification? do I need to writeRegister everytime?

        // check busy flag
        do {
            busyflag = 0x01 & readRegister(0x01);
        } while(busyflag == 1);

        // read LIDAR data
        distance = read16(0x10);
        printf("Distance: %d cm\n", distance);

        // reset busy flag
        busyflag = 1;
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}


static void PID() {

  int Kp = 1;
  int Ki = 1;
  int Kd = 1;
  float previous_error = 0.00;
  integral = 0.00;

  while(1)
  {
    float error = setpoint - distance;
    if (error > 1) {
      gpio_set_level(REDPIN, 0);
      gpio_set_level(GREENPIN, 0);
      gpio_set_level(BLUEPIN, 1);
    } else if (error < -1) {
      gpio_set_level(REDPIN, 1);
      gpio_set_level(GREENPIN, 0);
      gpio_set_level(BLUEPIN, 0);
      } else if (error < 1 || error > -1) {
      gpio_set_level(REDPIN, 0);
      gpio_set_level(GREENPIN, 1);
      gpio_set_level(BLUEPIN, 0);
    }
    integral = integral + error * dt;
    derivative = (error - previous_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    printf("PID found the following: \n\tError:\t\t%.0f \n\tIntegral:\t%.1f \n\tDerivative:\t%.1f \n\tOutput is: \t%.1f\n\n", error, integral, derivative, output);
    vTaskDelay(dt);
  }
}


void app_main() {

    led_init();
    i2c_master_init();
    i2c_scanner();
    xTaskCreate(PID,"PID", 4096, NULL, 5, NULL);
    xTaskCreate(LIDAR,"LIDAR", 4096, NULL, 5, NULL);

}

// // Configure timer
// static void alarm_init() {
//     // Select and initialize basic parameters of the timer
//     timer_config_t config;
//     config.divider = TIMER_DIVIDER;
//     config.counter_dir = TIMER_COUNT_UP;
//     config.counter_en = TIMER_PAUSE;
//     config.alarm_en = TIMER_ALARM_EN;
//     config.intr_type = TIMER_INTR_LEVEL;
//     config.auto_reload = TEST_WITH_RELOAD;
//     timer_init(TIMER_GROUP_0, TIMER_0, &config);
//
//     // Timer's counter will initially start from value below
//     timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
//
//     // Configure the alarm value and the interrupt on alarm
//     timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
//     timer_enable_intr(TIMER_GROUP_0, TIMER_0);
//     timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
//         (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
//
//     // Start timer
//     timer_start(TIMER_GROUP_0, TIMER_0);
// }

// // Default ID/color
// #define ID 3
// #define COLOR 'R'
//
// // Variables for my ID, minVal and status plus string fragments
// char start = 0x1B;
// char myID = (char) ID;
// char myColor = (char) COLOR;
// int len_out = 4;
// bool button_flag = false;
// // Mutex (for resources), and Queues (for button)
// SemaphoreHandle_t mux = NULL;
// static xQueueHandle gpio_evt_queue = NULL;
// static xQueueHandle timer_queue;
//
// // A simple structure to pass "events" to main task
// typedef struct {
//     int flag;     // flag for enabling stuff in timer task
// } timer_event_t;
//
// // System tags
// static const char *TAG_SYSTEM = "system";       // For debug logs
//
// // Button interrupt handler -- add to queue
// static void IRAM_ATTR gpio_isr_handler(void* arg){
//   uint32_t gpio_num = (uint32_t) arg;
//   xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
// }
//
// // ISR handler
// void IRAM_ATTR timer_group0_isr(void *para) {
//
//     // Prepare basic event data, aka set flag
//     timer_event_t evt;
//     evt.flag = 1;
//
//     // Yellow is shorter
//     if (myColor == 'G') {
//       timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_2_SEC * TIMER_SCALE);
//     }
//     else {
//       timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
//     }
//
//     // Clear the interrupt, Timer 0 in group 0
//     TIMERG0.int_clr_timers.t0 = 1;
//
//     // After the alarm triggers, we need to re-enable it to trigger it next time
//     TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
//
//     // Send the event data back to the main program task
//     xQueueSendFromISR(timer_queue, &evt, NULL);
// }
