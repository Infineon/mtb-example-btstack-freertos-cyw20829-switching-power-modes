/*******************************************************************************
 * File Name: app_bt_gatt_handler.c
 *
 * Description: This file contains the task that handles GATT events.
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

/*******************************************************************************
 * Header Files
 ******************************************************************************/
#include "app_bt_gatt_handler.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/

/**
 * @brief rate of change of battery level
 */
#define BATTERY_LEVEL_CHANGE (2)

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
/******************************************************************************
* Function Name: app_bt_gatt_req_read_by_type_handler
*******************************************************************************
* Summary:
*     This Function Process read-by-type request from peer device
*
* Parameters:
*     conn_id       Connection ID
*     opcode        Bluetooth LE GATT request type opcode
*     p_read_req    Pointer to read request containing the handle to read
*     len_requested length of data requested
*     p_error_handle Pointer to the error handle
*
* Return:
*  wiced_bt_gatt_status_t  Bluetooth LE GATT status
*
******************************************************************************/
wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                                   wiced_bt_gatt_opcode_t opcode,
                                                                   wiced_bt_gatt_read_by_type_t *p_read_req,
                                                                   uint16_t len_requested,
                                                                   uint16_t *p_error_handle)
{
    const gatt_db_lookup_table_t *puAttribute;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    uint16_t used = 0;

    if (p_rsp == NULL)
    {
        printf( "%s() No memory, len_requested: %d!!\r\n",
                   __func__, len_requested);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        *p_error_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle,
                                                        &p_read_req->uuid);

        if (attr_handle == 0)
            break;

        if ((puAttribute = app_bt_find_by_handle(attr_handle)) == NULL)
        {
            printf( "%s()  found type but no attribute for %d \r\n",
                       __func__, *p_error_handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used,
                                                                      len_requested - used,
                                                                      &pair_len,
                                                                      attr_handle,
                                                                      puAttribute->cur_len,
                                                                      puAttribute->p_data);
            if (filled == 0)
            {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        printf( "%s()  attr not found  start_handle: 0x%04x  "
                   "end_handle: 0x%04x  Type: 0x%04x\r\n",
                   __func__, p_read_req->s_handle, p_read_req->e_handle,
                   p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id,
                                               opcode,
                                               pair_len,
                                               used,
                                               p_rsp,
                                               (void *)&app_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/******************************************************************************
* Function Name: app_bt_gatt_req_read_multi_handler
*******************************************************************************
* Summary:
*     Process write read multi request from peer device
*
* Parameters:
*     conn_id       Connection ID
*     opcode        Bluetooth LE GATT request type opcode
*     p_read_req    Pointer to read request containing the handle to read
*     len_requested length of data requested
*     p_error_handle Pointer to the error handle
*
* Return:
*  wiced_bt_gatt_status_t  Bluetooth LE GATT status
*
******************************************************************************/
 wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id,
                                                                 wiced_bt_gatt_opcode_t opcode,
                                                                 wiced_bt_gatt_read_multiple_req_t *p_read_req,
                                                                 uint16_t len_requested,
                                                                 uint16_t *p_error_handle)
{
    const gatt_db_lookup_table_t *puAttribute;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint16_t used = 0;

    uint16_t handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);
    *p_error_handle = handle;

    if (p_rsp == NULL)
    {
        printf("line = %d fun = %s\n",__LINE__,__func__);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the
     * start and end handles */
    for (uint16_t iter = 0; iter < p_read_req->num_handles; iter++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, iter);
        *p_error_handle = handle;
        if ((puAttribute = app_bt_find_by_handle(handle)) == NULL)
        {
            printf( "%s()  no handle 0x%04x\r\n",
                       __func__, handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }

        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used,
                                                                    len_requested - used,
                                                                    puAttribute->handle,
                                                                    puAttribute->cur_len,
                                                                    puAttribute->p_data);
            if (!filled)
            {
                break;
            }
            used += filled;
        }
    }

    if (used == 0)
    {
        printf( "%s() no attr found\r\n", __func__);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                                            *p_read_req->p_handle_stream,
                                             WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id,
                                                opcode,
                                                used,
                                                p_rsp,
                                                (void *)&app_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}
 /******************************************************************************
 * Function Name: app_bt_alloc_buffer
 *******************************************************************************
 * Summary:
 *     This Function allocates the buffer of requested length
 *
 * Parameters:
 *     len            Length of the buffer
 *
 * Return:
 *  uint8_t*      pointer to allocated buffer
 *
  ******************************************************************************/
 uint8_t *app_bt_alloc_buffer(uint16_t len)
 {
     uint8_t *p = (uint8_t *)malloc(len);
     printf( "%s() len %d alloc %p \r\n", __FUNCTION__,len, p);
     return p;
 }

 /******************************************************************************
 * Function Name: app_bt_free_buffer
 *******************************************************************************
 * Summary:
 *     This Function frees the buffer requested
 *
 * Parameters:
 *     p_data            pointer to the buffer to be freed
 *
 * Return:
 *  void
 *
  ******************************************************************************/
 void app_bt_free_buffer(uint8_t *p_data)
 {
     if (p_data != NULL)
     {
         printf( "%s()        free:%p \r\n",__FUNCTION__, p_data);
         free(p_data);
     }
 }

 /******************************************************************************
 * Function Name: app_bt_gatt_req_read_handler
 *******************************************************************************
 * Summary:
 *     This Function Process read request from peer device
 *
 * Parameters:
 *     conn_id       Connection ID
 *     opcode        Bluetooth LE GATT request type opcode
 *     p_read_req    Pointer to read request containing the handle to read
 *     len_requested length of data requested
 *     p_error_handle Pointer to the error handle
 *
 * Return:
 *  wiced_bt_gatt_status_t  Bluetooth LE GATT status
 *
 ******************************************************************************/
  wiced_bt_gatt_status_t app_bt_gatt_req_read_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    const wiced_bt_gatt_read_t *p_read_req,
                                                    uint16_t len_requested,
                                                    uint16_t *p_error_handle)
 {
     gatt_db_lookup_table_t *puAttribute;
     uint16_t attr_len_to_copy;
     uint16_t to_send;
     uint8_t *from;

     *p_error_handle = p_read_req->handle;

     if ((puAttribute = app_bt_find_by_handle(p_read_req->handle)) == NULL)
     {
         printf( "%s()  Attribute not found, Handle: 0x%04x\r\n",
                     __func__, p_read_req->handle);
         wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                             WICED_BT_GATT_INVALID_HANDLE);
         return WICED_BT_GATT_INVALID_HANDLE;
     }

     attr_len_to_copy = puAttribute->cur_len;

     if (p_read_req->offset >= puAttribute->cur_len)
     {
         printf( "%s() offset:%d larger than attribute length:%d\r\n", __func__,
                    p_read_req->offset, puAttribute->cur_len);

         wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                             WICED_BT_GATT_INVALID_OFFSET);
         return WICED_BT_GATT_INVALID_OFFSET;
     }

     if(HDLC_BAS_BATTERY_LEVEL_VALUE == p_read_req->handle)
     {

         if (0 == app_bas_battery_level[0])
         {
             app_bas_battery_level[0] = 100;
         }
         else
         {
             app_bas_battery_level[0] = app_bas_battery_level[0] - BATTERY_LEVEL_CHANGE;
         }

         printf("\r\n================================================\r\n");
         printf( "Replying to read request, sending current Battery level: %u\r\n",
                                app_bas_battery_level[0]);
         printf("================================================\r\n");
     }
     to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);
     from = puAttribute->p_data + p_read_req->offset;
     return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */
 }

/******************************************************************************
 * Function Name: app_bt_find_by_handle
 *******************************************************************************
* Summary:
*   Find attribute description by handle
*
* Parameters:
*    handle    Handle to look up
*
* Return:
*   gatt_db_lookup_table_t   pointer containing handle data
*
******************************************************************************/
gatt_db_lookup_table_t *app_bt_find_by_handle(uint16_t handle)
{
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}
