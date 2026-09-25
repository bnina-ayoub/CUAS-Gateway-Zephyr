/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(can_node, LOG_LEVEL_DBG);

#define SLEEP_TIME_MS 10
#define UART_RX_PACKET_SIZE sizeof(joints_t)
#define PAN_GEAR_RATIO 4.0f // (80/20)
#define TILT_GEAR_RATIO 3.0f // (60/20)
#define PAN_VEL_TO_HZ_MULTIPLIER 254.7f * PAN_GEAR_RATIO
#define TILT_VEL_TO_HZ_MULTIPLIER 254.7f * TILT_GEAR_RATIO

static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec pan_dir = GPIO_DT_SPEC_GET(DT_ALIAS(pdir), gpios);
static const struct gpio_dt_spec tilt_dir = GPIO_DT_SPEC_GET(DT_ALIAS(tdir), gpios);
static const struct pwm_dt_spec pan_step = PWM_DT_SPEC_GET(DT_NODELABEL(pan_step));
static const struct pwm_dt_spec tilt_step = PWM_DT_SPEC_GET(DT_NODELABEL(tilt_step));

typedef struct {
	float pan_velocity;
	float tilt_velocity;
} joints_t;

joints_t joint_target;
volatile bool uart_packet_ready = false;
int8_t uart_packet[UART_RX_PACKET_SIZE];
volatile uint8_t uart_packet_len;

static void uart_callback(const struct device *dev, void *user_data)
{
    uart_irq_update(dev);

    while (uart_irq_is_pending(dev)) {
        if (!uart_irq_rx_ready(dev)) {
            break;
        }

        int8_t received[UART_RX_PACKET_SIZE];
        int count = uart_fifo_read(dev, received, sizeof(received));

        for (int i = 0; i < count; i++) {
            uart_packet[uart_packet_len++] = received[i];

            if (uart_packet_len == UART_RX_PACKET_SIZE) {
                memcpy(&joint_target, uart_packet, sizeof(joint_target));
                uart_packet_len = 0;
                uart_packet_ready = true;
                gpio_pin_toggle_dt(&green_led);
            }
        }
    }
}

int set_pan_velocity(float velocity, const struct gpio_dt_spec *dir_pin)
{
	/* 1. Deadband/Stop: Handle 0 velocity (or extremely small noise values) */
    if (fabsf(velocity) < 0.001f) {
		return pwm_set_dt(&pan_step, pan_step.period, 0);
    }
	
    /* If velocity > 0, set direction 1. If < 0, set direction 0 */
    gpio_pin_set_dt(dir_pin, (velocity > 0.0f) ? 1 : 0);
	
    /* 3. Calculate absolute frequency and hardware period */
    float abs_velocity = fabsf(velocity);
    float frequency_hz = abs_velocity * PAN_VEL_TO_HZ_MULTIPLIER;
    
    /* Calculate period in nanoseconds */
    uint32_t base_period_ns = (uint32_t)(1000000000.0f / frequency_hz);
	uint32_t period_ns = (uint32_t)(base_period_ns * 1.5f);
    uint32_t pulse_ns = period_ns / 2; /* 50% duty cycle for the STEP pulse */
	
    /* 4. Update the PWM hardware */
    return pwm_set_dt(&pan_step, period_ns, pulse_ns);
}

#define MIN_PWM_FREQ_HZ 10.0f

int set_tilt_velocity(float velocity, const struct gpio_dt_spec *dir_pin)
{
    /* 1. Calculate absolute frequency first */
	if (fabsf(velocity) < 0.001f) {
		return pwm_set_dt(&tilt_step, tilt_step.period, 0);
    }
	
    /* If velocity > 0, set direction 1. If < 0, set direction 0 */
    gpio_pin_set_dt(dir_pin, (velocity > 0.0f) ? 1 : 0);
	
    /* 3. Calculate absolute frequency and hardware period */
    float abs_velocity = fabsf(velocity);
    float frequency_hz = abs_velocity * PAN_VEL_TO_HZ_MULTIPLIER;
    
    /* 3. Set physical direction */
    gpio_pin_set_dt(dir_pin, (velocity > 0.0f) ? 1 : 0);
    
    /* 4. Calculate hardware period */
    uint32_t base_period_ns = (uint32_t)(1000000000.0f / frequency_hz);
    uint32_t period_ns = (uint32_t)(base_period_ns * 1.5f);
    uint32_t pulse_ns = period_ns / 2;
    
    /* 5. Update the PWM hardware */
    return pwm_set_dt(&tilt_step, period_ns, pulse_ns);
}

int main(void)
{
	int ret;
	const struct device *const uart_dev =
		DEVICE_DT_GET(DT_NODELABEL(flexcomm0_lpuart0));

	if (!gpio_is_ready_dt(&red_led) || !gpio_is_ready_dt(&green_led)) {
		printk("Error: LED device is not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
    return 0;
}
	
	ret = gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) return 0;

	if (!gpio_is_ready_dt(&pan_dir)) {
		printk("Error: Pan direction GPIO device is not ready\n");
		return 0;
	}
	
	ret = gpio_pin_configure_dt(&pan_dir, GPIO_OUTPUT_LOW);
	if (ret < 0) return 0;
	
	if (!gpio_is_ready_dt(&tilt_dir)) {
		printk("Error: Tilt direction GPIO device is not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&tilt_dir, GPIO_OUTPUT_LOW);
	if (ret < 0) return 0;

	if (!pwm_is_ready_dt(&pan_step)) {
		printk("Error: PWM device %s is not ready\n", pan_step.dev->name);
		return 0;
	}

	if (!pwm_is_ready_dt(&tilt_step)) {
		printk("Error: PWM device %s is not ready\n", tilt_step.dev->name);
		return 0;
	}

	if (!device_is_ready(uart_dev)) {
		printk("UART device is not ready\n");
		return 0;
	}

	ret = uart_irq_callback_user_data_set(uart_dev, uart_callback, NULL);
	if (ret != 0) {
		printk("Failed to set UART callback: %d\n", ret);
		return 0;
	}

	uart_irq_rx_enable(uart_dev);


	printk("Waiting for UART interrupt RX data...\n");

	while (1) {
		if (uart_packet_ready) {
			uart_packet_ready = false;
			printk("UART packet: pan_velocity=%f, tilt_velocity=%f\n",
				(double)joint_target.pan_velocity,
				(double)joint_target.tilt_velocity);
			set_pan_velocity(joint_target.pan_velocity, &pan_dir);
			set_tilt_velocity(joint_target.tilt_velocity, &tilt_dir);
		}

		}
	}