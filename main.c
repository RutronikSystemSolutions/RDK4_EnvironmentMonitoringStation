/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK4 Environment Monitoring Station
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2023-03-09
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "dps310_app.h"
#include "bmp581_app.h"
#include "bmi270_app.h"
#include "bme688_app.h"
#include "sht4x.h"
#include "sgp40.h"
#include "scd4x.h"
#include "sensirion_voc_algorithm.h"
#include "visiGenieSerial.h"
#include "TLE926x.h"
#include "led.h"

/*Priority for SBC interrupts*/
#define SBC_IRQ_PRIORITY		1
/*Priority for button interrupts*/
#define BUTTON_IRQ_PRIORITY		3
/*Priorities for sensor interrupts*/
#define MOTION_IRQ_PRIORITY		0
#define SENSOR_IRQ_PRIORITY		2
/*Arduino UART Baudrate*/
#define ARDU_BAUD_RATE       	9600

typedef struct app_sensor_data
{
	float dps_temperature;
	float dps_pressure;

	double bmp_temperature;
	double bmp_pressure;

	int16_t bmi_acc_x;
	int16_t bmi_acc_y;
	int16_t bmi_acc_z;

	int16_t bmi_gyr_x;
	int16_t bmi_gyr_y;
	int16_t bmi_gyr_z;

	float bme_temperature;
	float bme_pressure;
	float bme_humidity;
	float bme_gas_resistance;
	uint8_t bme_gas_index;

	uint16_t sgp_sraw_voc;
	int32_t sgp_voc_index;

	int32_t sht_temperature;
	int32_t sht_humidity;

	uint16_t scd_co2;
	int32_t scd_temperature;
	int32_t scd_humidity;
}sensor_data_t;

void btn_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
void motion_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void sensor_timer_isr(void *callback_arg, cyhal_timer_event_t event);
static cy_rslt_t sensor_timer_init(void);
void sbc_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
void hardware_init(void);
/*4D Systems display uart initialization*/
static cy_rslt_t ardu_uart_init();
/* UserApiConfig */
static bool uartAvailHandler(void);
static uint8_t uartReadHandler(void);
static void uartWriteHandler(uint32_t val);
static uint32_t uartGetMillis(void);
static void resetDisplay(void);
/* Display Event Handler */
static void myGenieEventHandler(void);
void delay_with_watchdog(uint32_t ms_100);
static void tick_timer_isr(void *callback_arg, cyhal_timer_event_t event);
static cy_rslt_t tick_timer_init(void);

/*Global interrupt flags*/
_Bool sensor_int_flag = false;
_Bool motion_int_flag = false;
_Bool sbc_int_flag = false;

/*A structure holding all the data gathered from the various sensors*/
sensor_data_t sensor_data_storage = {0};

/*VOC Index Algorithm Parameters*/
VocAlgorithmParams voc_algorithm_params;

/*User Button Interrupt Data*/
cyhal_gpio_callback_data_t btn_data =
{
		.callback = btn_interrupt_handler,
		.callback_arg = NULL,
};

/*Sensor Fusion Interrupt Data*/
cyhal_gpio_callback_data_t motion_int_data =
{
		.callback = motion_interrupt_handler,
		.callback_arg = NULL,
};

/*TLE9262 SBC Interrupt Data*/
cyhal_gpio_callback_data_t sbc_int_data =
{
		.callback = sbc_interrupt_handler,
		.callback_arg = NULL,
};

/*SBC Power Supply Variables*/
sbc_vcc3_on_t vcc3_supp;
sbc_vcc2_on_t vcc2_supp;

/*I2C Device Global Variables*/
cyhal_i2c_t I2C_scb2;
cyhal_i2c_cfg_t i2c_scb2_cfg =
{
		.is_slave = false,
	    .address = 0,
	    .frequencyhal_hz = 400000UL,
};

/*SCP Timer Object */
cyhal_timer_t sensors_timer;

/*SCP Timer Object */
cyhal_timer_t tick_timer;

/*System Ticks Global Variable*/
uint32_t ticks_cnt = 0;

/*Arduino UART object and configuration*/
cyhal_uart_t ardu_uart;
static UserApiConfig userConfig =
{
	.available = uartAvailHandler,
	.read =  uartReadHandler,
	.write = uartWriteHandler,
	.millis = uartGetMillis
};

int main(void)
{
    cy_rslt_t result;
    int8_t rslt = 0;
    uint8_t n_fields;
    static _Bool sensors_read = false;
    uint8_t second_cnt = 0;
    _Bool pready, tready = false;
    uint32_t waitPeriodDisplay = 0;
    uint16_t gaugeVal = 0;

    /*Variables for storing gauge values*/
    uint16_t last_temp = 0;
    uint16_t last_hum = 0;
    uint16_t last_press = 0;
    uint16_t last_co2 = 0;
    uint16_t last_aqi = 0;


    /*Hardware Initialization*/
    hardware_init();

    for (;;)
    {
		/*** Read the DPS310 data ***/
		result =  xensiv_dps3xx_check_ready(&dps310_sensor, &pready, &tready);
		if((result == CY_RSLT_SUCCESS) && pready &&  tready)
		{
			xensiv_dps3xx_read(&dps310_sensor, &sensor_data_storage.dps_pressure, &sensor_data_storage.dps_temperature);
		}

    	/*** Read the BMP581 data ***/
        rslt = bmp5_get_interrupt_status(&bmp5_status, &dev_bmp5);
        /* Read temperature and pressure data */
        if (bmp5_status & BMP5_INT_ASSERTED_DRDY)
        {
        	rslt = bmp5_get_sensor_data(&bmp5_data, &osr_odr_press_cfg, &dev_bmp5);
            if(rslt == BMP5_OK)
            {
                sensor_data_storage.bmp_temperature = bmp5_data.temperature;
                sensor_data_storage.bmp_pressure = bmp5_data.pressure;
            }
            /* NOTE : Read status register again to clear data ready interrupt status */
            (void)bmp5_get_interrupt_status(&bmp5_status, &dev_bmp5);
        }

        /*** Read the BME688 data ***/
        rslt = bme68x_get_data(BME68X_PARALLEL_MODE, bme_data, &n_fields, &bme);

        for (uint8_t i = 0; i < n_fields; i++)
        {
            if (bme_data[i].status == BME68X_VALID_DATA)
            {
            	sensor_data_storage.bme_temperature = bme_data[i].temperature;
            	sensor_data_storage.bme_humidity = bme_data[i].humidity;
            	sensor_data_storage.bme_pressure = bme_data[i].pressure;
            	sensor_data_storage.bme_gas_resistance = bme_data[i].gas_resistance;
            	sensor_data_storage.bme_gas_index = bme_data[i].gas_index;
            }
        }

        /* Measure & Read the SGP40 and SHT41 data  */
        if(sensor_int_flag )
        {
        	/*Feed the TLE9262 watchdog*/
        	if(!sbc_int_flag)
        	{
        		sbc_wd_trigger();
        	}

        	/*Start the measurements*/
            if(!sensors_read)
            {
            	sht4x_measure();
            	scd4x_set_ambient_pressure((uint16_t)(sensor_data_storage.bmp_pressure/100));
            	sgp40_measure_raw_with_rht(sensor_data_storage.sht_humidity, sensor_data_storage.sht_temperature);
            }
            /*Read the data*/
            else
            {
            	sht4x_read(&sensor_data_storage.sht_temperature, &sensor_data_storage.sht_humidity);
            	scd4x_read_measurement(&sensor_data_storage.scd_co2, &sensor_data_storage.scd_temperature, &sensor_data_storage.scd_humidity);
            	/*Read the raw data and process the VOC Index*/
            	sgp40_read_raw(&sensor_data_storage.sgp_sraw_voc);
            	VocAlgorithm_process(&voc_algorithm_params, sensor_data_storage.sgp_sraw_voc, &sensor_data_storage.sgp_voc_index);
            }
            sensors_read = !sensors_read;

            /*Clear the interrupt global variable*/
        	sensor_int_flag = false;

        	second_cnt++;
        }

        /*Store IMU data*/
        if(motion_int_flag )
        {
            /* Get accel and gyro data for x, y and z axis. */
            rslt = bmi270_get_sensor_data(bmi_sensor_data, 2, &bmi2_dev);
            if(rslt == BMI2_OK)
            {
            	sensor_data_storage.bmi_acc_x = bmi_sensor_data[ACCEL].sens_data.acc.x;
            	sensor_data_storage.bmi_acc_y = bmi_sensor_data[ACCEL].sens_data.acc.y;
            	sensor_data_storage.bmi_acc_z = bmi_sensor_data[ACCEL].sens_data.acc.z;
            	sensor_data_storage.bmi_gyr_x = bmi_sensor_data[GYRO].sens_data.gyr.x;
            	sensor_data_storage.bmi_gyr_y = bmi_sensor_data[GYRO].sens_data.gyr.y;
            	sensor_data_storage.bmi_gyr_z = bmi_sensor_data[GYRO].sens_data.gyr.z;
            }

            /*Clear the interrupt global variable*/
        	motion_int_flag = false;

            /*Clear the interrupt status*/
            (void)bmi2_get_int_status(&bmi_int_status, &bmi2_dev);
        }

        /*Print the info once per second*/
        if(second_cnt >= 9)
        {
        	printf("\x1b[2J\x1b[;H");
        	printf("   [ Sensor Fusion Adapter Board Data Output ]  \r\n");
        	printf("DPS310 --> T: %.2f deg C, P: %.2f Pa\r\n", sensor_data_storage.dps_temperature, sensor_data_storage.dps_pressure*100);
        	printf("BMP581 --> T: %.2f deg C, P: %.2f Pa\r\n", sensor_data_storage.bmp_temperature, sensor_data_storage.bmp_pressure);
        	printf("BME688 --> T: %.2f deg C, H: %.2f %%, P: %.2f Pa, Gas: %d, R: %.2f Ohm\r\n",
        			sensor_data_storage.bme_temperature,
					sensor_data_storage.bme_humidity,
					sensor_data_storage.bme_pressure,
					sensor_data_storage.bme_gas_index,
					sensor_data_storage.bme_gas_resistance);
        	printf("BMI270 --> accx: %d accy: %d accz: %d gyrx: %d gyry: %d gyrz: %d\r\n",
        			(unsigned int)sensor_data_storage.bmi_acc_x,
					(unsigned int)sensor_data_storage.bmi_acc_y,
					(unsigned int)sensor_data_storage.bmi_acc_z,
					(unsigned int)sensor_data_storage.bmi_gyr_x,
					(unsigned int)sensor_data_storage.bmi_gyr_y,
					(unsigned int)sensor_data_storage.bmi_gyr_z
					);
        	printf("SHT41  --> T: %.2f deg C, H: %.2f %%\r\n", (float)sensor_data_storage.sht_temperature/1000, (float)sensor_data_storage.sht_humidity/1000);
        	printf("SGP40  --> VOC: %d raw, VOC INDEX: %d\r\n", (int)sensor_data_storage.sgp_sraw_voc, (int)sensor_data_storage.sgp_voc_index);
            printf("SDC4X  --> CO2: %u ppm ", sensor_data_storage.scd_co2);
            printf("Temperature: %.2f\xB0 C ", (float)sensor_data_storage.scd_temperature/1000);
            printf("Humidity: %.2f%% RH\r\n", (float)sensor_data_storage.scd_humidity/1000);
        	second_cnt = 0;
        }

        /*Check for the display events*/
        genieDoEvents(true);

    	/*Update gauges once per 50 ms*/
    	if (uartGetMillis() >= waitPeriodDisplay)
    	{
    		/*SmartGauge0*/
    		gaugeVal = sensor_data_storage.sht_temperature/1000 + 40;
        	if(gaugeVal > 125){gaugeVal = 125;}
        	if(gaugeVal != last_temp)
        	{
        		last_temp = gaugeVal;
        		genieWriteObject(35, 0, last_temp);
        	}

    		/*SmartGauge1*/
    		gaugeVal = sensor_data_storage.sht_humidity/1000;
        	if(gaugeVal > 100){gaugeVal = 100;}
        	if(gaugeVal != last_hum)
        	{
        		last_hum = gaugeVal;
        		genieWriteObject(35, 1, last_hum);
        	}

    		/*SmartGauge2*/
    		gaugeVal = (uint16_t)sensor_data_storage.dps_pressure/10;
        	if(gaugeVal > 120){gaugeVal = 120;}
        	if(gaugeVal != last_press)
        	{
        		last_press = gaugeVal;
        		genieWriteObject(35, 2, last_press);
        	}

    		/*SmartGauge4*/
    		gaugeVal = sensor_data_storage.scd_co2;
        	if(gaugeVal > 5000){gaugeVal = 5000;}
        	if(gaugeVal != last_co2)
        	{
        		last_co2 = gaugeVal;
        		genieWriteObject(35, 4, last_co2);
        	}

    		/*SmartGauge5*/
    		gaugeVal = (uint16_t)sensor_data_storage.sgp_voc_index;
        	if(gaugeVal > 500){gaugeVal = 500;}
        	if(gaugeVal != last_aqi)
        	{
        		last_aqi = gaugeVal;
        		genieWriteObject(35, 5, last_aqi);
        	}

    		/*Increase the counter*/
    		waitPeriodDisplay = uartGetMillis() + 50;
    	}
    }
}

/* Interrupt handler callback function */
void btn_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);
}

/* Interrupt handler callback function */
void motion_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    /*Set the interrupt global flag*/
    motion_int_flag = true;
}

/*10 Hz interrupt for sensor measurement and reading control*/
static void sensor_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /*Set the interrupt global flag*/
    sensor_int_flag = true;
}

/*This function initialized the timer to generate 10Hz interrupt*/
static cy_rslt_t sensor_timer_init(void)
{
	 cy_rslt_t result;
	 const cyhal_timer_cfg_t scp_cfg =
	 {
	     .compare_value = 0,                 /* Timer compare value, not used */
	     .period = 999,                       /* Defines the timer period - 10 Hz */
	     .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
	     .is_compare = false,                /* Don't use compare mode */
	     .is_continuous = true,              /* Run the timer indefinitely */
	     .value = 0                          /* Initial value of counter */
	 };

	 result = cyhal_timer_init(&sensors_timer, NC, NULL);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_configure(&sensors_timer, &scp_cfg);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_set_frequency(&sensors_timer, 10000);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 cyhal_timer_register_callback(&sensors_timer, sensor_timer_isr, NULL);

	 cyhal_timer_enable_event(&sensors_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, SENSOR_IRQ_PRIORITY, true);

	 result =  cyhal_timer_start(&sensors_timer);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 return result;
}

/* Interrupt handler callback function */
void sbc_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    sbc_int_flag = true;
    SBC_ISR();
    sbc_int_flag = false;
}

void hardware_init(void)
{
    cy_rslt_t result;
    SBC_ErrorCode sbc_err;
    int16_t error = 0;
    uint16_t serial_number[3];

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /*Initialize the LED*/
    result = initialize_led();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize Tick Timer*/
    result = tick_timer_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize SBC Interrupt Pin*/
    result = cyhal_gpio_init(INT_SBC, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    /*Register callback functions */
    cyhal_gpio_register_callback(INT_SBC, &sbc_int_data);
    /* Enable rising edge interrupt events */
    cyhal_gpio_enable_event(INT_SBC, CYHAL_GPIO_IRQ_RISE, SBC_IRQ_PRIORITY, true);

    /*SBC Initializations*/
    sbc_err = sbc_init();
    if(sbc_err.flippedBitsMask)
    {
    	printf("SBC initialization failure.\r\n");
    	CY_ASSERT(0);
    }

    /*Turn ON the 5V Power Supply VCC2 for the Display */
    vcc2_supp = VCC2_ON_ALWAYS;
    sbc_err = sbc_switch_vcc2(vcc2_supp);
    if(sbc_err.flippedBitsMask)
    {
    	printf("Could not enable the VCC2.\r\n");
    	CY_ASSERT(0);
    }

    /*Turn ON the 3.3V Power Supply VCC3 for the Arduino Shield(s) */
    vcc3_supp = VCC3_ENABLED;
    sbc_err = sbc_switch_vcc3(vcc3_supp);
    if(sbc_err.flippedBitsMask)
    {
    	printf("Could not enable the VCC3.\r\n");
    	CY_ASSERT(0);
    }

    /*SBC Watchdog Configuration*/
    sbc_configure_watchdog(TIME_OUT_WD, NO_WD_AFTER_CAN_LIN_WAKE, WD_1000MS);
    Cy_SysLib_Delay(100);


    /*Initialize Buttons*/
    result = cyhal_gpio_init(USER_BUTTON, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Register callback functions */
    cyhal_gpio_register_callback(USER_BUTTON, &btn_data);

    /* Enable falling edge interrupt events */
    cyhal_gpio_enable_event(USER_BUTTON, CYHAL_GPIO_IRQ_FALL, BUTTON_IRQ_PRIORITY, true);

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize I2C Master*/
    result = cyhal_i2c_init(&I2C_scb2, I2C_SDA, I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&I2C_scb2, &i2c_scb2_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Initialize Sensor Fusion Board Interrupt*/
    result = cyhal_gpio_init(ARD_IO2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    /*Register callback function */
    cyhal_gpio_register_callback(ARD_IO2, &motion_int_data);
    /* Enable rising edge interrupt events */
    cyhal_gpio_enable_event(ARD_IO2, CYHAL_GPIO_IRQ_BOTH, MOTION_IRQ_PRIORITY, true);

    /*Initialize the DPS310 Sensor*/
    result = dps310_app_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("DPS310 Sensor Failure\n\r");
    	CY_ASSERT(0);
    }

    /*Initialize the BMI270 Sensor*/
    result = bmi270_app_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize the BMP581 Sensor*/
    result = bmp581_app_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize the BME688 Sensor*/
    result = bme688_app_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Check the SHT4x sensor*/
    if(sht4x_probe() != SHT4X_STATUS_OK)
    {
    	printf("SHT4x Sensor Failure\n\r");
    	CY_ASSERT(0);
    }

    /*Check the SGP40 sensor*/
    if(sgp40_probe()  != SGP40_STATUS_OK)
    {
    	printf("SGP40 Sensor Failure\n\r");
    	CY_ASSERT(0);
    }

    /*VOC Index Algorithm Stack Initialization*/
    VocAlgorithm_init(&voc_algorithm_params);

    /*Initialize Sensors Timer*/
    result = sensor_timer_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initiate the SCD4x sensor*/
    scd4x_wake_up();
    scd4x_stop_periodic_measurement();
    scd4x_reinit();
    error = scd4x_get_serial_number(&serial_number[0], &serial_number[1], &serial_number[2]);
    if (error)
    {
    	printf("SCD4x Sensor Failure\n\r");
    	CY_ASSERT(0);
    }
    else
    {
        printf("SCD4x serial: 0x%04x%04x%04x\n\r", serial_number[0], serial_number[1], serial_number[2]);
        error = scd4x_start_periodic_measurement();
        if(error)
        {
        	printf("Could not start SCD4x periodic measurements\n\r");
        	CY_ASSERT(0);
        }
    }

    /*Initialize Display RESET pin*/
    result = cyhal_gpio_init(ARD_IO8, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize The Arduino UART*/
    result = ardu_uart_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize ViSi-Genie*/
    genieInitWithConfig(&userConfig);
    genieAttachEventHandler(myGenieEventHandler);
    resetDisplay();
    delay_with_watchdog(1000);
    genieWriteContrast(14);
    delay_with_watchdog(1000);
    genieWriteStr(0, GENIE_VERSION);
    delay_with_watchdog(2000);
}

static cy_rslt_t ardu_uart_init(void)
{
	cy_rslt_t result;
	uint32_t actualbaud;

    /* Initialize the UART configuration structure */
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL,
        .rx_buffer_size = 0
    };

    /* Initialize the UART Block */
    result = cyhal_uart_init(&ardu_uart, ARD_UART_TX, ARD_UART_RX, NC, NC, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	result = cyhal_uart_set_baud(&ardu_uart, ARDU_BAUD_RATE, &actualbaud);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	/*Connect internal pull-up resistor*/
	cyhal_gpio_configure(ARD_UART_RX, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP);

	return result;
}

/* UserApiConfig Handlers */
static bool uartAvailHandler(void)
{
  return (_Bool)cyhal_uart_readable(&ardu_uart);
}

static uint8_t uartReadHandler(void)
{
	uint8_t byte;

	(void)cyhal_uart_getc(&ardu_uart, &byte,0xFFFFFFFF);

	return byte;
}

static void uartWriteHandler(uint32_t val)
{
	(void)cyhal_uart_putc(&ardu_uart,val);
}

static uint32_t uartGetMillis(void)
{
	return ticks_cnt;
}

static void resetDisplay(void)
{
	cyhal_gpio_write(ARD_IO8, false);
	delay_with_watchdog(500);
	cyhal_gpio_write(ARD_IO8, true);
	delay_with_watchdog(3000);
}

/*ViSi Genie Event Handler*/
static void myGenieEventHandler(void)
{
  GenieFrame Event;
  genieDequeueEvent(&Event);
  led_data_t led_data = {LED_ON, LED_MAX_BRIGHTNESS};
  static uint32_t last_brightness = 0;
  uint16_t slider_pos = 0;

  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
	    if (Event.reportObject.object == GENIE_OBJ_SLIDER)
	    {
	    	slider_pos = Event.reportObject.data_lsb;
	    	slider_pos |= Event.reportObject.data_msb << 8;
	    	led_data.brightness = (slider_pos * 100) / 100;
	    	if(last_brightness != led_data.brightness)
	    	{
	    		last_brightness = led_data.brightness;
	    		update_led_state(&led_data);
	    	}
	    }
  }
}

void delay_with_watchdog(uint32_t ms_100)
{
	uint32_t delay_cnt = ms_100/100;

	if(!delay_cnt)
	{
    	/*Feed the watchdog*/
    	if(!sbc_int_flag)
    	{
    		sbc_wd_trigger();
    	}
    	return;
	}

	for(uint32_t i = 0; i < delay_cnt; i++)
	{
    	/*Feed the watchdog*/
    	if(!sbc_int_flag)
    	{
    		sbc_wd_trigger();
    	}

    	/*Delay 100 ms*/
		Cy_SysLib_Delay(100);
	}
}

/*1000 Hz interrupt for tick counter */
static void tick_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    ticks_cnt++;
}

/*This function initialized the timer to generate 1000Hz interrupt*/
static cy_rslt_t tick_timer_init(void)
{
	 cy_rslt_t result;
	 const cyhal_timer_cfg_t scp_cfg =
	 {
	     .compare_value = 0,                 /* Timer compare value, not used */
	     .period = 999,                       /* Defines the timer period - 1000 Hz */
	     .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
	     .is_compare = false,                /* Don't use compare mode */
	     .is_continuous = true,              /* Run the timer indefinitely */
	     .value = 0                          /* Initial value of counter */
	 };

	 result = cyhal_timer_init(&tick_timer, NC, NULL);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_configure(&tick_timer, &scp_cfg);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_set_frequency(&tick_timer, 1000000);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 cyhal_timer_register_callback(&tick_timer, tick_timer_isr, NULL);

	 cyhal_timer_enable_event(&tick_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, SENSOR_IRQ_PRIORITY, true);

	 result =  cyhal_timer_start(&tick_timer);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 return result;
}

/* [] END OF FILE */
