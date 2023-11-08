#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

/*Node Identifiers by Device Tree Alias*/
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN0 DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS0 DT_GPIO_FLAGS(LED0_NODE, gpios)

#define LED1 DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN1 DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS1 DT_GPIO_FLAGS(LED1_NODE, gpios)

#define STACK_SIZE 2000
struct k_thread coop_thread;
K_THREAD_STACK_DEFINE(coop_stack, STACK_SIZE);

int counter;
bool led_is_on;

/*************PID CODE************/
/************PID STRUCT********/
typedef struct {
    float kp;
    float ki;
    float kd;

    float tau;

    float max_lim_out;
    float min_lim_out;

    float sample_period;

    float integral_term;
    float error_prev;
    float differential_term;
    float meas_prev;

    float out;
} Controller;

void controller_initialize(Controller *pid){
    pid->integral_term = 0.0f;
    pid->error_prev = 0.0f;
    pid->differential_term = 0.0f;
    pid->meas_prev = 0.0f;
    pid->out = 0.0f;
}

float controller_update(Controller *pid, float setpoint, float meas_val){
float error_val = setpoint - meas_val;
/*Proportional*/
float proportional_term = pid->kp*error_val;
/*Integral*/
pid->integral_term = 0.5f *pid->ki * pid->sample_period * (error_val + pid->error_prev) + pid->integral_term;
/*Derivative*/
pid->differential_term = -(2.0f*pid->kd * (meas_val - pid->meas_prev) 
                            + (2.0f * pid->tau - pid->sample_period) * pid->differential_term )
                        / (2.0f * pid->tau + pid->sample_period);
/*TODO: Integrator Antiwindup*/
/*Output*/
pid->out = proportional_term + pid->integral_term + pid->differential_term;
/*Clamp Output*/
if (pid->out > pid->max_lim_out){pid->out = pid->max_lim_out;}
else if (pid->out < pid->min_lim_out){pid->out = pid->min_lim_out;}
/*Store current vals as prev vals*/
pid->error_prev = error_val;
pid->meas_prev = meas_val;

return pid->out;
}
/*********************/

void thread_entry_func(void){

   const struct device *dev;
   dev = device_get_binding(LED1);
   bool led_is_on = true;
   int ret = gpio_pin_configure(dev, PIN1, GPIO_OUTPUT_ACTIVE | FLAGS1);

   struct k_timer led_timer;
   k_timer_init(&led_timer, NULL, NULL);

   while(1){
    counter = counter+1;
    led_is_on = !led_is_on;
    gpio_pin_set(dev, PIN1, (int)led_is_on);
    k_timer_start(&led_timer, K_MSEC(2000), K_NO_WAIT);
    k_timer_status_sync(&led_timer);
   } 
}

void main(void)
{
    const struct device *dev;
    led_is_on = true;
    int ret;

    dev = device_get_binding(LED0);
    
    k_thread_create
    (
        &coop_thread,
        coop_stack,
        STACK_SIZE,
        (k_thread_entry_t) thread_entry_func,
        NULL,
        NULL,
        NULL,
        K_PRIO_COOP(7),
        0,
        K_NO_WAIT
    );
    if(dev == NULL){
        return;
    }
    ret = gpio_pin_configure(dev, PIN0, GPIO_OUTPUT_ACTIVE | FLAGS0);
    if(ret < 0)
    {
        return;
    }

    while(1){
        led_is_on = !led_is_on;
        gpio_pin_set(dev, PIN0, (int)led_is_on);
        k_msleep(500);
    }
}
