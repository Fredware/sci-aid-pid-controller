#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

/***************** Status LED Configuration ****************/
#define LED0_NODE DT_ALIAS(led0)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN0 DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS0 DT_GPIO_FLAGS(LED0_NODE, gpios)

#define LED1_NODE DT_ALIAS(led1)
#define LED1 DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN1 DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS1 DT_GPIO_FLAGS(LED1_NODE, gpios)

int counter;
bool led_is_on;

/******************** PWM Configuration ********************/
#define PWM_LABEL DT_PROP(DT_CHILD(DT_NODELABEL(timers2), pwm), label)

/******************** I2C Configuration ********************/
#define I2C_LABEL DT_PROP(DT_NODELABEL(i2c1), label)

/******************** ADC Configuration ********************/
#define ADC_LABEL DT_PROP(DT_NODELABEL(adc1), label)

/******************** PID Configuration ********************/
/********** PID Thread **********/
#define PID_STACK_SIZE 2000
K_THREAD_STACK_DEFINE(pid_stack, PID_STACK_SIZE);
struct k_thread pid_thread;

/********** PID Struct **********/
typedef struct
{
    float kp;
    float ki;
    float kd;

    float tau;

    float out_lim_max;
    float out_lim_min;

    float sample_period;

    float integral_term;
    float error_prev;
    float differential_term;
    float meas_prev;

    float out;
} Controller;

/********** PID Functions **********/
void controller_initialize(Controller *pid)
{
    pid->integral_term = 0.0f;
    pid->error_prev = 0.0f;
    pid->differential_term = 0.0f;
    pid->meas_prev = 0.0f;
    pid->out = 0.0f;
}

float controller_update(Controller *pid, float setpoint, float meas_val)
{
    float error_val = setpoint - meas_val;

    /*PID*/
    float proportional_term = pid->kp * error_val;
    pid->integral_term = 0.5f * pid->ki * pid->sample_period * (error_val + pid->error_prev) + pid->integral_term;
    pid->differential_term = -(2.0f * pid->kd * (meas_val - pid->meas_prev) + (2.0f * pid->tau - pid->sample_period) * pid->differential_term) / (2.0f * pid->tau + pid->sample_period);
    /*TODO: Integrator Antiwindup*/

    /*Output*/
    pid->out = proportional_term + pid->integral_term + pid->differential_term;

    /*Clamp Output*/
    if (pid->out > pid->out_lim_max)
    {
        pid->out = pid->out_lim_max;
    }
    else if (pid->out < pid->out_lim_min)
    {
        pid->out = pid->out_lim_min;
    }

    /*Store current vals as prev vals*/
    pid->error_prev = error_val;
    pid->meas_prev = meas_val;

    return pid->out;
}
/*********************/

/********** Thread Entry Functions *********/
void pid_thread_entry(void)
{
    const struct device *dev_i2c;
    dev_i2c = device_get_binding(I2C_LABEL);
    uint32_t i2c_config = I2C_SPEED_SET(I2C_SPEED_FAST_PLUS) | I2C_MODE_MASTER;
    i2c_configure(dev_i2c, i2c_config);

    const struct device *dev_pwm;
    dev_pwm = device_get_binding(PWM_LABEL);

    const struct device* dev_adc;
    dev_adc = device_get_binding(ADC_LABEL);

    const struct device *dev_led;
    dev_led = device_get_binding(LED1);
    int ret = gpio_pin_configure(dev_led, PIN1, GPIO_OUTPUT_ACTIVE | FLAGS1);
    gpio_pin_set(dev_led, PIN1, (int)true);

    if (!device_is_ready(dev_i2c))
    {
        /*Turn off status LED if I2C is not ready*/
        gpio_pin_set(dev_led, PIN1, (int)false);
        k_msleep(500);
    };

    struct k_timer led_timer;
    k_timer_init(&led_timer, NULL, NULL);

    while (1)
    {
        counter = counter + 1;
        led_is_on = !led_is_on;
        gpio_pin_set(dev_led, PIN1, (int)led_is_on);
        k_timer_start(&led_timer, K_MSEC(2000), K_NO_WAIT);
        k_timer_status_sync(&led_timer);
    }
}

/******************** Main Loop ********************/
void main(void)
{
    const struct device *dev_led;
    led_is_on = true;
    dev_led = device_get_binding(LED0);
    int ret = gpio_pin_configure(dev_led, PIN0, GPIO_OUTPUT_ACTIVE | FLAGS0);
    gpio_pin_set(dev_led, PIN0, (int)true);

    /**/
    k_thread_create(
        &pid_thread,
        pid_stack,
        PID_STACK_SIZE,
        (k_thread_entry_t)pid_thread_entry,
        NULL,
        NULL,
        NULL,
        K_PRIO_COOP(7),
        0,
        K_NO_WAIT);
}
