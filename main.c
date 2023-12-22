/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the Bluetooth LE
*              Switching Power Modes
* Example for ModusToolbox.The battery service exposes the battery level of the
* device and supports enabling battery level notifications.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/

/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_gpio.h"
#include "app_sleep.h"

#include "app_bt_event_handler.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/


/**
 * @brief rate of change of battery level
 */
#define BATTERY_LEVEL_CHANGE (2)


#define USR_BUTTON_STATE_DOWN       0
#define USR_BUTTON_STATE_UNKNOWN    2


/* Stack size for Battery Client BTN task */
#define BTN_TASK_STACK_SIZE                 (512u)

/* Task Priority of Battery CLient BTN Task */
#define BTN_TASK_PRIORITY                   (2)

#define GPIO_INTERRUPT_PRIORITY              7


/*******************************************************************************
*        Variable Definitions
*******************************************************************************/
/**
 * @brief FreeRTOS variable to store handle of task created to update and send dummy
   values of temperature
 */
TaskHandle_t bas_task_handle;

/* Handle of the button task */
TaskHandle_t button_handle;

/* sec_timer is a periodic timer that ticks every 5 Seconds */
TimerHandle_t sec_timer;

/* ms_timer is a periodic timer that ticks every millisecond */
TimerHandle_t ms_timer;
/**
 * @brief variable to track the current power mode
 */
volatile uint8_t power_modes = 0;

typedef struct
{
    uint8_t   usr_button_state;
    uint8_t   usr_button2_state;
} battery_server_state_t;

/* Holds the global state of the battery client application */
static battery_server_state_t battery_server_state;


/*Enum For changing the Power modes*/
enum PowerModes{

    SYSPM_ACTIVE = 0U, /* Indicate to system in Active mode */
    SYSPM_SLEEP,       /* Indicate to system in Sleep mode */
    SYSPM_DEEPSLEEP,   /* Indicate to system in Deepsleep mode */
    SYSPM_DEEPSLEEP_RAM,/* Indicate to system in Deepsleep-Ram mode */
    SYSPM_HIBERNATE,   /* Indicate to system in Hibernate mode */
};

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/


/* GATT Event Callback Functions */

/* Task to send notifications with dummy battery values */
void bas_task(void *pvParam);
/* HAL timer callback registered when timer reaches terminal count */
void bas_timer_callb(TimerHandle_t timer_handle);
void ms_timer_callb(TimerHandle_t timer_handle);
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void gpio2_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
void battery_server_btn_interrupt_config(void);
void button_task                         (void *arg);
void battery_server_application_init(void);
static void adv_start();



cyhal_gpio_callback_data_t btn_cb_data =
    {
        .callback     = gpio_interrupt_handler,
        .callback_arg = NULL
    };
cyhal_gpio_callback_data_t btn2_cb_data =
    {
        .callback     = gpio2_interrupt_handler,
        .callback_arg = NULL
    };

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/******************************************************************************
* Function Name: main
*******************************************************************************
* Summary:
*     Entry point to the application. Set device configuration and start BT
*     stack initialization.  The actual application initialization will happen
*     when stack reports that BT device is ready
*
* Parameters:
*  void
*
* Return:
*  void
*
 ******************************************************************************/
int main()
{
    cy_rslt_t cy_result;
    wiced_result_t  result;
    BaseType_t rtos_result;

    /* Initialize the board support package */
    cy_result = cybsp_init();
    if (CY_RSLT_SUCCESS != cy_result)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    printf("================================================\n\n");
    printf("========CYW20829 Free RTOS Low Power CE ========\r\n");
    printf("================================================\n");

    /* Initialising the HCI transport for Host contol */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);


    /* Register call back and configuration with stack */
    result = wiced_bt_stack_init(app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS != result)
    {
        printf( "Bluetooth Stack Initialization failed!! \r\n");
        CY_ASSERT(0);
    }

    rtos_result = xTaskCreate(bas_task, "BAS Task", (configMINIMAL_STACK_SIZE * 4),
                                    NULL, (configMAX_PRIORITIES - 3), &bas_task_handle);
    if(pdPASS == rtos_result)
    {
        printf("BAS task created successfully\n");
    }
    else
    {
        printf("BAS task creation failed\n");
        CY_ASSERT(0);
    }
    /* Button handling */
    rtos_result =xTaskCreate(button_task, "Button task", BTN_TASK_STACK_SIZE, NULL,
                              BTN_TASK_PRIORITY, &button_handle);

    if(pdPASS == rtos_result)
    {
        printf("Button task created successfully\n");
    }
    else
    {
        printf("Button task creation failed\n");
        CY_ASSERT(0);
    }

    sec_timer = xTimerCreate("bas_timer", pdMS_TO_TICKS(5000), pdTRUE, NULL, bas_timer_callb);

    /* Timer init failed. Stop program execution */
    if (NULL == sec_timer)
    {
        printf("BAS Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
    else
    {
         printf("BAS Timer created successfully \r\n");
    }

    ms_timer = xTimerCreate("ms_timer", pdMS_TO_TICKS(1), pdTRUE, NULL, ms_timer_callb);
    /* Timer init failed. Stop program execution */
    if (NULL == sec_timer)
    {
        printf("ms_timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
    else
    {
         printf("ms_timer created successfully \r\n");
    }

    xTimerStart(ms_timer, 0);


    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);
}


/******************************************************************************
* Function Name: bas_timer_callb
*******************************************************************************
* Summary:
*    This callback function is invoked on timeout of 1 second timer
*
* Parameters:
*  TimerHandle_t: unused
*
* Return:
*  void
*
 ******************************************************************************/
void bas_timer_callb(TimerHandle_t timer_handle)
{
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(bas_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
* Function Name: onems_timer_callb
*******************************************************************************
* Summary:
*    This callback function is invoked on timeout of 1 ms timer and it will
*    not allow device to go DeepSleep
*
* Parameters:
*  TimerHandle_t: unused
*
* Return:
*  void
*
 ******************************************************************************/
void ms_timer_callb(TimerHandle_t timer_handle)
{
    static uint32_t ms_timer =0;

    ms_timer++;
}

/******************************************************************************
* Function Name: bas_task
*******************************************************************************
* Summary:
*    This task updates dummy battery value every time it is notified
*         and sends a notification to the connected pee
*
* Parameters:
*  void*: unused
*
* Return:
*  void
*
 ******************************************************************************/
void bas_task(void *pvParam)
{
    while(true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* Battery level is read from gatt db and is reduced by 2 percent
        * by default and initialized again to 100 once it reaches 0*/
        if (0 == app_bas_battery_level[0])
        {
            app_bas_battery_level[0] = 100;
        }
        else
        {
            app_bas_battery_level[0] = app_bas_battery_level[0] - BATTERY_LEVEL_CHANGE;
        }

        if (bt_conn_id)
        {
            if (app_bas_battery_level_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION)
            {
                wiced_bt_gatt_server_send_notification(bt_conn_id,
                                                    HDLC_BAS_BATTERY_LEVEL_VALUE,
                                                    app_bas_battery_level_len,
                                                    app_bas_battery_level,NULL);
                printf("\r\n================================================\r\n");
                printf( "Sending Notification: Battery level: %u\r\n",
                        app_bas_battery_level[0]);
                printf("================================================\r\n");
            }
        }
    }
}



/******************************************************************************
* Function Name: app_bt_batt_level_init
*******************************************************************************
* Summary:
*   This function Starts the timer for updating Battery Level
*
* Parameters:
*    IsStartStop  Decides to start or stop the timer
*
* Return:
*   void
*
******************************************************************************/
void app_bt_batt_level_init(bool IsStartStop)
{
    /* Start the timer */
    if(IsStartStop)
    {
        xTimerStart(sec_timer, 5000);

    }
    else
    {
        xTimerStop(sec_timer, 5000);
    }

}

/******************************************************************************
 * Function Name: battery_server_application_init
*******************************************************************************
 * Summary:
 *   This function handles application level initialization tasks and is called from the BT
 *   management callback once the LE stack enabled event (BTM_ENABLED_EVT) is triggered
 *   This function is executed in the BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *   None
 *
 * Return:
 *  None
 *
******************************************************************************/
void battery_server_application_init(void)
{
    /*
     * Interrupt configuration for User Button
     */
    battery_server_btn_interrupt_config();

    /* Prompt user */
    printf("======================================================================\n");
    printf("| Press the User Button:                                             |\n");
    printf("|   Press User Button 1 to change power mode                         |\n");
    printf("|   Press User Button 2 to start/stop adv                            |\n");
    printf("======================================================================\n");
}

/******************************************************************************
 * Function Name: battery_server_btn_interrupt_config
*******************************************************************************
 * Summary:
 *   This function initializes a pin as input that triggers interrupt on
 *   falling edges.
 *
 * Parameters:
 *   None
 *
 * Return:
 *  None
 *
******************************************************************************/
void battery_server_btn_interrupt_config(void)
{

    /* Initialize the user button 1 and 2 */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYBSP_USER_BTN_DRIVE, CYBSP_BTN_OFF);
    cyhal_gpio_init(CYBSP_USER_BTN2, CYHAL_GPIO_DIR_INPUT, CYBSP_USER_BTN_DRIVE, CYBSP_BTN_OFF);
    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &btn_cb_data);
    cyhal_gpio_register_callback(CYBSP_USER_BTN2, &btn2_cb_data);
    /* Enable for FALL */
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, GPIO_INTERRUPT_PRIORITY, true);
    cyhal_gpio_enable_event(CYBSP_USER_BTN2, CYHAL_GPIO_IRQ_FALL, GPIO_INTERRUPT_PRIORITY, true);
    /* Set button state to unknown */
    battery_server_state.usr_button_state  = USR_BUTTON_STATE_UNKNOWN;
    battery_server_state.usr_button2_state = USR_BUTTON_STATE_UNKNOWN;

}

/*******************************************************************************
* Function Name: gpio2_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
* Return
*  None
*
*******************************************************************************/
static void gpio2_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    battery_server_state.usr_button2_state = USR_BUTTON_STATE_DOWN;

    if(Active)
    {
        xTimerStop(ms_timer, 0);
        Active =false;
    }

    vTaskNotifyGiveFromISR(button_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
* Return
*  None
*
*******************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    battery_server_state.usr_button_state = USR_BUTTON_STATE_DOWN;

    if(Active)
    {
        xTimerStop(ms_timer, 0);
        Active =false;
    }


    vTaskNotifyGiveFromISR(button_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*******************************************************************************
* Function Name: adv_start
********************************************************************************
* Summary:
*    Start Undirected Bluetooth LE Advertisements
*
* Parameters:
*    None
*
* Return
*  None
*
*******************************************************************************/
static void adv_start()
{
    wiced_result_t result;

    /* Start Undirected Bluetooth LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */

    if((bt_conn_id != 0) || (app_bt_adv_conn_state == ADVERTISEMENT))
    {
        printf( "Device is in either Connected state or in Advertisement state\r\n");
    }
    else
    {
        result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
        if (WICED_BT_SUCCESS != result)
        {
            printf( "Advertisement cannot start because of error: %d \r\n",
                       result);
            CY_ASSERT(0);
        }
        printf("\r\n================================================\r\n");
        printf("** Discover device with Low Power 20829 name **\r\n");
        printf("================================================\r\n\n");
    }
}

/*******************************************************************************
* Function Name: button_task
********************************************************************************
* Summary:
*    To transition the system to MCU active,sleep,deep sleep,
*    deep sleep-ram and hibernate power states and Start the Advertisements
*    depends on the button press
* Parameters:
*  None
*
* Return
*  None
*
*******************************************************************************/
void button_task(void *arg)
{
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE,  portMAX_DELAY);

        if(battery_server_state.usr_button2_state == USR_BUTTON_STATE_DOWN)
        {
            battery_server_state.usr_button2_state  =USR_BUTTON_STATE_UNKNOWN;
            adv_start();
        }
        if (battery_server_state.usr_button_state == USR_BUTTON_STATE_DOWN)
        {
            battery_server_state.usr_button_state = USR_BUTTON_STATE_UNKNOWN;
            power_modes++;
            if(power_modes > SYSPM_HIBERNATE)
            {
                power_modes =0;
            }
            /* Change the Power Mode */
            switch(power_modes)
            {
            case SYSPM_ACTIVE:
                printf("Normal mode\r\n");
                break;
            case SYSPM_SLEEP:
                printf("Going to Sleep\r\n");
                cyhal_syspm_lock_deepsleep();
                break;
            case SYSPM_DEEPSLEEP:
                printf("Going to DeepSleep\r\n");
                Cy_SysPm_SetDeepSleepMode(CY_SYSPM_MODE_DEEPSLEEP);
                cyhal_syspm_unlock_deepsleep();
                break;
           case SYSPM_DEEPSLEEP_RAM:
                 printf("Going to DeepSleep-Ram\r\n");
                 Cy_SysPm_SetDeepSleepMode(CY_SYSPM_MODE_DEEPSLEEP_RAM);
                 break;
            case SYSPM_HIBERNATE:
                printf("Going to Hibernate\r\n");
                Cy_SysLib_Delay(1000/* msec */);
                cyhal_syspm_hibernate(CYHAL_SYSPM_HIBERNATE_PINA_LOW);
                break;
            default:
                break;
            }
        }
    }
}

/* [] END OF FILE */
