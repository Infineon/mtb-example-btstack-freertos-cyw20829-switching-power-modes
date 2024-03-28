/*******************************************************************************
 * File Name: app_bt_event_handler.h
 *
 * Description: This file is the public interface of app_bt_event_handler.c
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 ******************************************************************************/

#ifndef APP_BT_EVENT_HANDLER_H_
#define APP_BT_EVENT_HANDLER_H_

/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cycfg_gap.h"
#include <string.h>
#include "cybt_platform_trace.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "GeneratedSource/cycfg_bt_settings.h"
#include "app_bt_utils.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_memory.h"
#include "wiced_bt_stack.h"
#include "cycfg_bt_settings.h"
#include "wiced_bt_l2c.h"
#include "cyabs_rtos.h"
#include "stdlib.h"
#include <inttypes.h>
#include "app_sleep.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>
#include "cyhal_wdt.h"
#include "cybsp_bt_config.h"

/*******************************************************************************
 * Structures
 *******************************************************************************/

/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/
/**
 * @brief This enumeration combines the advertising, connection states from two
 *        different callbacks to maintain the status in a single state variable
 */
/**
 * @brief variable to track connection and advertising state
 */
typedef enum
{
    DISCONNECTION_IDLE,
    ADVERTISEMENT,
    CONNECTION_IDLE
} device_state_t;
extern device_state_t app_bt_adv_conn_state ;

extern bool Active;
extern uint16_t                    bt_conn_id;                 /* Host BT Connection ID */
extern uint8_t                     bt_peer_addr[BD_ADDR_LEN];  /* Host BT address */
/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
 wiced_bt_gatt_status_t app_bt_connect_event_handler(wiced_bt_gatt_connection_status_t *p_conn_status);
 wiced_bt_gatt_status_t app_bt_server_event_handler(wiced_bt_gatt_event_data_t *p_data,
                                                    uint16_t *p_error_handle);
 wiced_bt_gatt_status_t app_bt_gatt_event_callback(wiced_bt_gatt_evt_t event,
                                                    wiced_bt_gatt_event_data_t *p_event_data);
 wiced_bt_gatt_status_t app_bt_set_value(uint16_t attr_handle,
                                         const uint8_t *p_val,
                                         uint16_t len);

/* Callback function for Bluetooth stack management type events */
wiced_bt_dev_status_t  app_bt_management_callback(wiced_bt_management_evt_t event,
                                                    wiced_bt_management_evt_data_t *p_event_data);
 wiced_bt_gatt_status_t app_bt_write_handler(wiced_bt_gatt_event_data_t *p_data,
                                                    uint16_t *p_error_handle);

 void                   app_bt_init                           (void);
 extern void battery_server_application_init(void);
 void                   app_bt_batt_level_init                (bool IsStartStop);



#endif /* APP_BT_EVENT_HANDLER_H_ */
