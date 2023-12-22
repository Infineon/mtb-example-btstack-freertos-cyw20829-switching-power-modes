/*******************************************************************************
 * File Name: app_bt_event_handler.c
 *
 * Description: This file contains the bluetooth event handler that processes
 *              the bluetooth events from host stack.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
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
 ******************************************************************************/

/*******************************************************************************
 * Header Files
 ******************************************************************************/
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"

/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/
/**
 * @brief variable to track connection and advertising state
 */
device_state_t app_bt_adv_conn_state = DISCONNECTION_IDLE;

/**
 * @brief variable to make system in active
 */
bool Active = true;

/**
 * @brief Typdef for function used to free allocated buffer to stack
 */
typedef void (*pfn_free_buffer_t)(uint8_t *);


uint16_t                    bt_conn_id;                 /* Host BT Connection ID */
uint8_t                     bt_peer_addr[BD_ADDR_LEN];  /* Host BT address */
/*******************************************************************************
 * Function Definitions
 ******************************************************************************/

/******************************************************************************
* Function Name: app_bt_management_callback
*******************************************************************************
* Summary:
*  This is a Bluetooth stack event handler function to receive management events
*  from the Bluetooth stack and process as per the application.
*
* Parameters:
*  wiced_bt_management_evt_t       Bluetooth LE event code of one byte length
*  wiced_bt_management_evt_data_t  Pointer to Bluetooth LE management event
*                                  structures
*
* Return:
*  wiced_result_t Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
 ******************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result  = WICED_BT_ERROR;
    wiced_bt_device_address_t bda = {0};
    const wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;
    wiced_bt_dev_encryption_status_t *p_status = NULL;
    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */

        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Initialize the application */
            wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            printf( "Local Bluetooth Address: ");
            print_bd_address(bda);

            /* Perform application-specific initialization */
            app_bt_init();
            result = WICED_BT_SUCCESS;
        }
        else
        {
            printf( "Failed to initialize Bluetooth controller and stack \r\n");
        }
        /* Registering callback for system power management */
        create_cpu_sleep_cb();
        create_deep_sleep_cb();
        create_deep_sleep_ram_cb();
        create_hibernate_cb();
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        printf("* TM_USER_CONFIRMATION_REQUEST_EVT: Numeric_value = %"PRIu32" *\r",p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
        result = WICED_BT_SUCCESS;
        break;

    case BTM_PASSKEY_NOTIFICATION_EVT:
        printf("\r\n  PassKey Notification from BDA: ");
        print_bd_address(p_event_data->user_passkey_notification.bd_addr);
        printf("PassKey: %"PRIu32" \n", p_event_data->user_passkey_notification.passkey );
        result = WICED_BT_SUCCESS;
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        printf( "  BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT\r\n");
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        result = WICED_BT_SUCCESS;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        printf( "  Pairing Complete: %d ",p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        result = WICED_BT_SUCCESS;
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* Local identity Keys Update */
        result = WICED_BT_SUCCESS;
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* Local identity Keys Request */
        result = WICED_BT_ERROR;
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* Paired Device Link Keys update */
        result = WICED_BT_SUCCESS;
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        result = WICED_BT_ERROR;
        break;


    case BTM_ENCRYPTION_STATUS_EVT:
        p_status = &p_event_data->encryption_status;
        printf( "  Encryption Status Event for : bd ");
        print_bd_address(p_status->bd_addr);
        printf( "  res: %d \r\n", p_status->result);
        result = WICED_BT_SUCCESS;
        break;

    case BTM_SECURITY_REQUEST_EVT:
        printf( "  BTM_SECURITY_REQUEST_EVT\r\n");
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                    WICED_BT_SUCCESS);
        result = WICED_BT_SUCCESS;
        break;

    case BTM_BLE_CONNECTION_PARAM_UPDATE:
        printf( "BTM_BLE_CONNECTION_PARAM_UPDATE \r\n");
        printf( "ble_connection_param_update.bd_addr: ");
        print_bd_address(p_event_data->ble_connection_param_update.bd_addr);
        printf( "ble_connection_param_update.conn_interval       : %d\r\n",p_event_data->ble_connection_param_update.conn_interval);
        printf( "ble_connection_param_update.conn_latency        : %d\r\n",p_event_data->ble_connection_param_update.conn_latency);
        printf( "ble_connection_param_update.supervision_timeout : %d\r\n",p_event_data->ble_connection_param_update.supervision_timeout);
        printf( "ble_connection_param_update.status              : %d\r\n\n",p_event_data->ble_connection_param_update.status);
        result = WICED_BT_SUCCESS;
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

        /* Advertisement State Changed */
        p_adv_mode = &p_event_data->ble_advert_state_changed;
        printf( "Advertisement State Change: %s\r\n",get_bt_advert_mode_name(*p_adv_mode));

        if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
        {
            /* Advertisement Stopped */
            printf( "Advertisement stopped\r\n");

            /* Check connection status after advertisement stops */
            if (bt_conn_id == 0)
            {
                app_bt_adv_conn_state = DISCONNECTION_IDLE;
            }
            else
            {
                app_bt_adv_conn_state = CONNECTION_IDLE;
            }
        }
        else
        {
            /* Advertisement Started */
            printf( "Advertisement started\r\n");
            app_bt_adv_conn_state = ADVERTISEMENT;
        }

        result = WICED_BT_SUCCESS;
        break;

    default:
        printf( "Unhandled Bluetooth Management Event: 0x%x %s\r\n",
                   event, get_bt_event_name(event));
        break;
    }

    return result;
}

/******************************************************************************
* Function Name: app_bt_init
*******************************************************************************
* Summary:
*     This function handles application level initialization tasks and is
*     called from the BT management callback once the Bluetooth LE stack
*     enabled event (BTM_ENABLED_EVT) is triggered This function is executed
*     in the BTM_ENABLED_EVT management callback.
*
* Parameters:
*  void
*
* Return:
*  void
*
 ******************************************************************************/
void app_bt_init(void)
{

    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    battery_server_application_init();

    /* Disable pairing for this application */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                            cy_bt_adv_packet_data);

    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(app_bt_gatt_event_callback);
    printf( "GATT event Handler registration status: %s \r\n",
               get_bt_gatt_status_name(status));

    /* Initialize GATT Database */
    status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf( "GATT database initialization status: %s \r\n",
               get_bt_gatt_status_name(status));
}

/******************************************************************************
* Function Name: app_bt_gatt_event_callback
*******************************************************************************
* Summary:
*    This Function handles the all the GATT events - GATT Event Handler
*
* Parameters:
*     event              Bluetooth LE GATT event type
*     p_event_data       Pointer to Bluetooth LE GATT event data
*
* Return:
*  wiced_bt_gatt_status_t  Bluetooth LE GATT status
*
 ******************************************************************************/
 wiced_bt_gatt_status_t app_bt_gatt_event_callback(wiced_bt_gatt_evt_t event,
                                                         wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    uint16_t error_handle = 0;

    /* Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event
     * parameters to the callback function */
    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = app_bt_connect_event_handler (&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        status = app_bt_server_event_handler (p_event_data,&error_handle);
        break;
        /* GATT buffer request, typically sized to max of bearer mtu - 1 */
    case GATT_GET_RESPONSE_BUFFER_EVT:
        p_event_data->buffer_request.buffer.p_app_rsp_buffer = app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
        p_event_data->buffer_request.buffer.p_app_ctxt = (void *)&app_bt_free_buffer;
        status = WICED_BT_GATT_SUCCESS;
        break;
        /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
    case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            status = WICED_BT_GATT_SUCCESS;
        }
        break;

    default:
        printf( " Unhandled GATT Event \r\n");
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}

/******************************************************************************
* Function Name: app_bt_connect_event_handler
*******************************************************************************
* Summary:
*    This callback function handles connection status changes
*
* Parameters:
*     p_conn_status              Pointer to data that has connection details
*
* Return:
*  wiced_bt_gatt_status_t See possible status codes in wiced_bt_gatt_status_e
* in wiced_bt_gatt.h
*
 ******************************************************************************/
 wiced_bt_gatt_status_t app_bt_connect_event_handler (wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
            /* Device has connected */
            printf( "Connected : BDA ");
            print_bd_address(p_conn_status->bd_addr);
            printf( "Connection ID '%d'\r\n", p_conn_status->conn_id);

            /* Store the connection ID and peer BD Address */
            bt_conn_id = p_conn_status->conn_id;
            memcpy(bt_peer_addr, p_conn_status->bd_addr, BD_ADDR_LEN);

            /* Update the adv/conn state */
            app_bt_adv_conn_state = CONNECTION_IDLE;
            /* Save BT connection ID in application data structure */
            bt_conn_id = p_conn_status->conn_id;
            /* Save BT peer ADDRESS in application data structure */
            memcpy(bt_peer_addr, p_conn_status->bd_addr, BD_ADDR_LEN);
        }
        else
        {
            /* Device has disconnected */
            printf( "Disconnected : BDA ");
            print_bd_address(p_conn_status->bd_addr);
            printf( "Connection ID '%d', Reason '%s'\r\n", p_conn_status->conn_id,
                       get_bt_gatt_disconn_reason_name(p_conn_status->reason));

            /* Set the connection id to zero to indicate disconnected state */
            bt_conn_id = 0;
            app_bt_batt_level_init(0);
        }

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/******************************************************************************
* Function Name: app_bt_server_event_handler
*******************************************************************************
* Summary:
*    The callback function is invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs
*         in GATT Event handler function. GATT Server Event Callback function
*
* Parameters:
*    p_data   Pointer to Bluetooth LE GATT request data
*    p_error_handle Pointer to the error handle
*
* Return:
*  wiced_bt_gatt_status_t  Bluetooth LE GATT status
*
 ******************************************************************************/

wiced_bt_gatt_status_t
app_bt_server_event_handler (wiced_bt_gatt_event_data_t *p_data,uint16_t *p_error_handle)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_attribute_request_t   *p_att_req = &p_data->attribute_request;

    switch (p_att_req->opcode)
    {
    /* Attribute read notification (attribute value internally read from GATT database) */
    case GATT_REQ_READ:
    case GATT_REQ_READ_BLOB:
        status = app_bt_gatt_req_read_handler(p_att_req->conn_id, p_att_req->opcode,
                                              &p_att_req->data.read_req,
                                              p_att_req->len_requested,
                                              p_error_handle);
        break;

    case GATT_REQ_READ_BY_TYPE:
        status = app_bt_gatt_req_read_by_type_handler(p_att_req->conn_id, p_att_req->opcode,
                                                      &p_att_req->data.read_by_type,
                                                      p_att_req->len_requested,
                                                      p_error_handle);
        break;

    case GATT_REQ_READ_MULTI:
    case GATT_REQ_READ_MULTI_VAR_LENGTH:
        status = app_bt_gatt_req_read_multi_handler(p_att_req->conn_id, p_att_req->opcode,
                                                    &p_att_req->data.read_multiple_req,
                                                    p_att_req->len_requested,
                                                    p_error_handle);
        break;

    case GATT_REQ_WRITE:
    case GATT_CMD_WRITE:
    case GATT_CMD_SIGNED_WRITE:
        status = app_bt_write_handler(p_data, p_error_handle);
        if ((p_att_req->opcode == GATT_REQ_WRITE) && (status == WICED_BT_GATT_SUCCESS))
        {
            const wiced_bt_gatt_write_req_t *p_write_request = &p_att_req->data.write_req;
            wiced_bt_gatt_server_send_write_rsp(p_att_req->conn_id, p_att_req->opcode,
                                                p_write_request->handle);
        }
        break;

    case GATT_REQ_PREPARE_WRITE:
        status = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_REQ_EXECUTE_WRITE:
        wiced_bt_gatt_server_send_execute_write_rsp(p_att_req->conn_id, p_att_req->opcode);
        status = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_REQ_MTU:
        /* Application calls wiced_bt_gatt_server_send_mtu_rsp() with the desired mtu */
        status = wiced_bt_gatt_server_send_mtu_rsp(p_att_req->conn_id,
                                                   p_att_req->data.remote_mtu,
                                                   wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
        printf( "    Set MTU size to: %d  status: 0x%d\r\n",
                    p_att_req->data.remote_mtu, status);
        break;

    case GATT_HANDLE_VALUE_CONF: /* Value confirmation */
        status = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_HANDLE_VALUE_NOTIF:
        status = WICED_BT_GATT_SUCCESS;
        break;

    default:
        printf( "  %s() Unhandled Event opcode: %d\r\n",
                   __func__, p_att_req->opcode);
        break;
    }
    return status;
}



/******************************************************************************
* Function Name: app_bt_write_handler
*******************************************************************************
* Summary:
*   The function is invoked when GATTS_REQ_TYPE_WRITE is received from the
*   client device and is invoked GATT Server Event Callback function. This
*   handles "Write Requests" received from Client device.
*
* Parameters:
*    p_write_req   Pointer to Bluetooth LE GATT write request
*    p_error_handle Pointer to the error handle
*
* Return:
*   wiced_bt_gatt_status_t  Bluetooth LE GATT status
*
******************************************************************************/

 wiced_bt_gatt_status_t app_bt_write_handler(wiced_bt_gatt_event_data_t *p_data,uint16_t *p_error_handle)
{
    wiced_bt_gatt_write_req_t *p_write_req = &p_data->attribute_request.data.write_req;

    *p_error_handle = p_write_req->handle;

    CY_ASSERT(( NULL != p_data ) && (NULL != p_write_req));

    return app_bt_set_value(p_write_req->handle,
                                    p_write_req->p_val,
                                    p_write_req->val_len);

}

/******************************************************************************
* Function Name: app_bt_set_value
*******************************************************************************
* Summary:
*   The function is invoked by app_bt_write_handler to set a value
*   to GATT DB.
*
* Parameters:
*    attr_handle  GATT attribute handle
*    p_val        Pointer to Bluetooth LE GATT write request value
*    len          length of GATT write request
*
* Return:
*   wiced_bt_gatt_status_t  Bluetooth LE GATT status
*
******************************************************************************/

wiced_bt_gatt_status_t app_bt_set_value(uint16_t attr_handle,
                                        const uint8_t *p_val,
                                        uint16_t len)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        /* Check for a matching handle entry */
        if ((app_gatt_db_ext_attr_tbl[i].handle == attr_handle) &&
            (app_gatt_db_ext_attr_tbl[i].max_len >= len))
        {
            /* Detected a matching handle in the external lookup table */
            /* Value fits within the supplied buffer; copy over the value */
            app_gatt_db_ext_attr_tbl[i].cur_len = len;
            memset(app_gatt_db_ext_attr_tbl[i].p_data, 0x00, app_gatt_db_ext_attr_tbl[i].max_len);
            memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, app_gatt_db_ext_attr_tbl[i].cur_len);

            if (memcmp(app_gatt_db_ext_attr_tbl[i].p_data, p_val, app_gatt_db_ext_attr_tbl[i].cur_len) == 0)
            {
                status = WICED_BT_GATT_SUCCESS;
            }

            if( (app_gatt_db_ext_attr_tbl[i].handle == HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG) &&
                (GATT_CLIENT_CONFIG_NOTIFICATION == app_bas_battery_level_client_char_config[0]) )
            {
                printf( "Battery Server Notifications Enabled \r\n");
                /* Start battery level timer */
                app_bt_batt_level_init(1);
            }
            else
            {
                printf( "Battery Server Notifications Disabled \r\n");
            }
            break;
        }
        else
        {
            /* Value to write will not fit within the table */
            status = WICED_BT_GATT_INVALID_ATTR_LEN;
            printf( "Invalid attribute length\r\n");
        }
    }
    if (WICED_BT_GATT_SUCCESS != status)
    {
        printf( "%s() FAILED %d \r\n", __func__, status);
    }
    return status;
}
