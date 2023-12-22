/*******************************************************************************
 * File Name: app_sleep.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for handling device sleep
 *
 *******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 *                              INCLUDES
 ******************************************************************************/

#include <stdio.h>
#include "cy_retarget_io.h"
#include "app_sleep.h"
#include "cyabs_rtos_dsram.h"
#include "cyhal_gpio.h"

/*******************************************************************************
 *                              Macro Definitions
 *******************************************************************************/


/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

cy_stc_syspm_callback_params_t syspm_cpu_sleep_params;
cy_stc_syspm_callback_params_t syspm_deep_sleep_params;
cy_stc_syspm_callback_params_t syspm_hibernate_params;
cy_stc_syspm_callback_params_t syspm_dsram_params;

extern cy_stc_syspm_warmboot_entrypoint_t syspmBspDeepSleepEntryPoint;

cy_stc_syspm_callback_t syspm_cpu_sleep_cb_handler =
{
    syspm_cpu_sleep_cb,
    CY_SYSPM_SLEEP,
    0u,
    &syspm_cpu_sleep_params,
    NULL,
    NULL,
    253
};

cy_stc_syspm_callback_t syspm_deep_sleep_cb_handler =
{
    syspm_ds_cb,
    CY_SYSPM_DEEPSLEEP,
    0u,
    &syspm_deep_sleep_params,
    NULL,
    NULL,
    253
};

cy_stc_syspm_callback_t syspm_dsram_cb_handler =
{
    syspm_dsram_cb,
    CY_SYSPM_DEEPSLEEP_RAM,
    0u,
    &syspm_dsram_params,
    NULL,
    NULL,
    0
};

cy_stc_syspm_callback_t syspm_hibernate_handler =
{
    syspm_hibernate_cb,
    CY_SYSPM_HIBERNATE,
    0u,
    &syspm_hibernate_params,
    NULL,
    NULL,
    253
};


/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: syspm_cpu_sleep_cb
********************************************************************************
* Summary:
*    Cpu Sleep Callback Function
*
* Parameters:
*    callbackParams: Pointer to cy_stc_syspm_callback_params_t
*    mode: cy_en_syspm_callback_mode_t
*
* Return
*    cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
*
*******************************************************************************/
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
syspm_cpu_sleep_cb( cy_stc_syspm_callback_params_t *callbackParams,
                cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    /* Template CB function for SysPm driver and can be customised for use cases */
    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

/*******************************************************************************
* Function Name: syspm_ds_cb
********************************************************************************
* Summary:
*    DeepSleep Callback Function
*
* Parameters:
*    callbackParams: Pointer to cy_stc_syspm_callback_params_t
*    mode: cy_en_syspm_callback_mode_t
*
* Return
*    cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
*
*******************************************************************************/
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t syspm_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
                                 cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    /* Template CB function for SysPm driver and can be customised for use cases */
    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
            /*CTS and RTS pins are configured as analog hign-z as it is connected to kitprog3
             * and also drawing more current */
            cyhal_gpio_configure(CYBSP_BT_UART_RTS, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_ANALOG);
            cyhal_gpio_configure(CYBSP_BT_UART_CTS, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_ANALOG);
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after exiting the low power mode */
        {
            /*CTS and RTS pins are configured to it's default configuration */
            cyhal_gpio_configure(CYBSP_BT_UART_RTS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE);
            cyhal_gpio_configure(CYBSP_BT_UART_CTS, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULL_NONE);
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

/*******************************************************************************
* Function Name: syspm_hibernate_cb
********************************************************************************
* Summary:
*    Hibernate Callback Function
*
* Parameters:
*    callbackParams: Pointer to cy_stc_syspm_callback_params_t
*    mode: cy_en_syspm_callback_mode_t
*
* Return
*    cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
*
*******************************************************************************/
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
syspm_hibernate_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    /* Template CB function for SysPm driver and can be customised for use cases */
    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

/*******************************************************************************
* Function Name: syspm_dsram_cb
********************************************************************************
* Summary:
*     DeepSleep-RAM Callback Function
*
* Parameters:
*    callbackParams: Pointer to cy_stc_syspm_callback_params_t
*    mode: cy_en_syspm_callback_mode_t
*
* Return
*    cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
*
*******************************************************************************/
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
syspm_dsram_cb( cy_stc_syspm_callback_params_t *callbackParams,
                cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    /* Template CB function for SysPm driver and can be customised for use cases */
    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
            /*CTS and RTS pins are configured as analog hign-z as it is connected to kitprog3
             * and also drawing more current */
            cyhal_gpio_configure(CYBSP_BT_UART_RTS, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_ANALOG);
            cyhal_gpio_configure(CYBSP_BT_UART_CTS, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_ANALOG);
            Cy_Syslib_SetWarmBootEntryPoint((uint32_t*)&syspmBspDeepSleepEntryPoint, false);
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after entering the low power mode */
        {
            if(Cy_SysLib_IsDSRAMWarmBootEntry())
            {
            cy_retarget_io_deinit();
            cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,CY_RETARGET_IO_BAUDRATE);
            /*CTS and RTS pins are configured to it's default configuration */
            cyhal_gpio_configure(CYBSP_BT_UART_RTS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE);
            cyhal_gpio_configure(CYBSP_BT_UART_CTS, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULL_NONE);
            }
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

/*******************************************************************************
* Function Name: create_cpu_sleep_cb
********************************************************************************
* Summary:
*     Creates a syspm Callback for CPU Sleep mode
*
* Parameters:
*    void
*
* Return
*   void
*
*******************************************************************************/
void create_cpu_sleep_cb(void)
{
    Cy_SysPm_RegisterCallback(&syspm_cpu_sleep_cb_handler);
}

/*******************************************************************************
* Function Name: create_deep_sleep_cb
********************************************************************************
* Summary:
*     Creates a syspm Callback for Deep Sleep mode
*
* Parameters:
*    void
*
* Return
*   void
*
*******************************************************************************/
void create_deep_sleep_cb(void)
{
    Cy_SysPm_RegisterCallback(&syspm_deep_sleep_cb_handler);
}

/*******************************************************************************
* Function Name: create_deep_sleep_ram_cb
********************************************************************************
* Summary:
*     Creates a syspm Callback for DS-RAM mode
*
* Parameters:
*    void
*
* Return
*   void
*
*******************************************************************************/
void create_deep_sleep_ram_cb(void)
{
     Cy_SysPm_RegisterCallback(&syspm_dsram_cb_handler);
}

/*******************************************************************************
* Function Name: create_hibernate_cb
********************************************************************************
* Summary:
*     Creates a syspm Callback for Hibernate mode
*
* Parameters:
*    void
*
* Return
*   void
*
*******************************************************************************/
void create_hibernate_cb(void)
{
    Cy_SysPm_RegisterCallback(&syspm_hibernate_handler);
}

