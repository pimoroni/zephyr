/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/rtio/rtio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/printk.h>


#define LED_ON_TIME 100 // heartbeat LED on duration in milliseconds
#define LED_FREQUENCY 5 // heartbeat LED frequency in seconds

const struct device *const dev = DEVICE_DT_GET_ANY(bosch_bme280);

SENSOR_DT_READ_IODEV(iodev, DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bme280),
		{SENSOR_CHAN_AMBIENT_TEMP, 0},
		{SENSOR_CHAN_HUMIDITY, 0},
		{SENSOR_CHAN_PRESS, 0});

RTIO_DEFINE(ctx, 1, 1);


#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

struct gpio_dt_spec *current_led = &led1;

//#define LED1_NODE DT_ALIAS(led1)
//static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

bool led_state = true;

#define SERVICE_DATA_LEN        9
#define SERVICE_UUID            0xfcd2      /* BTHome service UUID */
#define IDX_TEMPL               4           /* Index of lo byte of temp in service data*/
#define IDX_TEMPH               5           /* Index of hi byte of temp in service data*/

#define IDX_HUML                7
#define IDX_HUMH                8

#define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, \
				  BT_GAP_ADV_SLOW_INT_MIN, \
				  BT_GAP_ADV_SLOW_INT_MAX, NULL)



#define DELAY 2000000
#define ALARM_CHANNEL_ID 0

static uint8_t service_data[SERVICE_DATA_LEN] = {
	BT_UUID_16_ENCODE(SERVICE_UUID),
	0x40,
	0x02,	/* Temperature */
	0xc4,	/* Low byte */
	0x00,   /* High byte */
	0x03,	/* Humidity */
	0xbf,	/* 50.55%  low byte*/
	0x13,   /* 50.55%  high byte*/
};

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_SVC_DATA16, service_data, ARRAY_SIZE(service_data))
};

struct k_timer my_timer;

static void timer_timeout(struct k_timer *timer_id)
{
	int ret = gpio_pin_set_dt(current_led, 0);
	k_sleep(K_MSEC(LED_ON_TIME));
	ret = gpio_pin_set_dt(current_led, 1);
	(void)ret;
}


static const struct device *check_bme280_device(void)
{
	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
	err = bt_le_adv_start(ADV_PARAM, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
}

void led_flash_fatal(void) {
	current_led = &led0;
	k_timer_stop(&my_timer);
	gpio_pin_set_dt(&led1, 1);
	k_timer_start(&my_timer, K_MSEC(100), K_MSEC(100));
	while (1){
		k_sleep(K_MSEC(BT_GAP_ADV_SLOW_INT_MIN));
	};
}

int main(void)
{
	//const struct device *const counter_dev = DEVICE_DT_GET(TIMER);
	int err;

	/*if (!gpio_is_ready_dt(&led1)) {
		printk("LED: device not ready.\n");
		return 0;
	}

	err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		return 0;
	}
	gpio_pin_set_dt(&led1, 1);*/

	if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1)) {
		printk("LED: device not ready.\n");
		return 0;
	}

	err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		printk("LED: device failed.\n");
		return 0;
	}

	printk("Starting BTHome sensor template\n");
	k_timer_init(&my_timer, timer_timeout, NULL);
	k_timer_start(&my_timer, K_MSEC(10), K_MSEC(10));

	const struct device *dev = check_bme280_device();

	if (dev == NULL) {
		led_flash_fatal();
	}

	k_timer_stop(&my_timer);
	k_timer_start(&my_timer, K_SECONDS(LED_FREQUENCY), K_SECONDS(LED_FREQUENCY));

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	/*bt_addr_le_t addr[CONFIG_BT_ID_MAX];
	char saddr[BT_ADDR_LE_STR_LEN];
	size_t count;
	bt_id_get(addr, &count);
	bt_addr_le_to_str(&addr[0], saddr, sizeof(saddr));*/

	//printf("Bluetooth Mac: %s\n", saddr);

	for (;;) {
		uint8_t buf[128];

		int rc = sensor_read(&iodev, &ctx, buf, 128);

		if (rc != 0) {
			printk("%s: sensor_read() failed: %d\n", dev->name, rc);
			led_flash_fatal();
		}

		const struct sensor_decoder_api *decoder;

		rc = sensor_get_decoder(dev, &decoder);

		uint32_t temp_fit = 0;
		struct sensor_q31_data temp_data = {0};

		decoder->decode(buf,
			(struct sensor_chan_spec) {SENSOR_CHAN_AMBIENT_TEMP, 0},
			&temp_fit, 1, &temp_data);

		uint32_t press_fit = 0;
		struct sensor_q31_data press_data = {0};

		decoder->decode(buf,
				(struct sensor_chan_spec) {SENSOR_CHAN_PRESS, 0},
				&press_fit, 1, &press_data);

		uint32_t hum_fit = 0;
		struct sensor_q31_data hum_data = {0};

		decoder->decode(buf,
				(struct sensor_chan_spec) {SENSOR_CHAN_HUMIDITY, 0},
				&hum_fit, 1, &hum_data);

		printk("temp: %s%d.%d; press: %s%d.%d; humidity: %s%d.%d\n",
			PRIq_arg(temp_data.readings[0].temperature, 6, temp_data.shift),
			PRIq_arg(press_data.readings[0].pressure, 6, press_data.shift),
			PRIq_arg(hum_data.readings[0].humidity, 6, hum_data.shift));

		int itemp = (uint32_t)__PRIq_arg_get_int(temp_data.readings[0].temperature, temp_data.shift);
		int ftemp = (uint32_t)__PRIq_arg_get_frac(temp_data.readings[0].temperature, 2, temp_data.shift);
		int16_t temp = (itemp * 100) + ftemp;

		int ihum = (uint32_t)__PRIq_arg_get_int(temp_data.readings[0].humidity, hum_data.shift);
		int fhum = (uint32_t)__PRIq_arg_get_frac(temp_data.readings[0].humidity, 2, hum_data.shift);
		int16_t hum = (ihum * 100) + fhum;

		/* Simulate temperature from 0C to 25C */
		service_data[IDX_TEMPH] = temp >> 8;
		service_data[IDX_TEMPL] = temp & 0xff;
	
		service_data[IDX_HUMH] = hum >> 8;
		service_data[IDX_HUML] = hum & 0xff;
	
		err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
		if (err) {
			printk("Failed to update advertising data (err %d)\n", err);
		} else {
			printk("Updated advertising data (temp %d.%d, %lld)\n", itemp, ftemp, temp);
		}
		//gpio_pin_toggle_dt(&led1);
		//k_sleep(K_MSEC(BT_GAP_ADV_SLOW_INT_MIN));
		k_sleep(K_MSEC(1600 * 30));
	}
	return 0;
}
