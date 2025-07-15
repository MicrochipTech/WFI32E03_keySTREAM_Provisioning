/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/
/*******************************************************************************
Copyright (C) 2020 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system/net/sys_net.h"
#include "ktaFieldMgntHook.h"
#include "cloud_status.h"
#include "atca_iface.h"
#include "atca_basic.h"
#include "app_mqtt.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
static SYS_MODULE_OBJ g_netServHandle = SYS_MODULE_OBJ_INVALID;
extern SYS_MODULE_OBJ g_sSysMqttHandle;

APP_DATA g_appData;

#define KEYSTREAM_NO_OPS_INTERVAL       (1000)
#define KEYSTREAM_NO_OPS_DELAY_COUNTER  (30)
TKktaKeyStreamStatus ktaKSCmdStatus = E_K_KTA_KS_STATUS_NONE;
enum cloud_iot_state kta_gCloudWifiState = CLOUD_STATE_UNKNOWN;

#define APP_PRINT_BUFFER_SIZ    2048
static char printBuff[APP_PRINT_BUFFER_SIZ] __attribute__((aligned(4)));
static int printBuffPtr;
static OSAL_MUTEX_HANDLE_TYPE consoleMutex;
#define RECV_BUFFER_LEN		1400  
uint8_t recv_buffer[RECV_BUFFER_LEN + 1];

static uint32_t g_lastPubTimeout = 0;
#define MQTT_PERIOIDC_PUB_TIMEOUT   5 //Sec ; if the value is 0, Periodic Publish will disable
#define MQTT_PUB_TIMEOUT_CONST (MQTT_PERIOIDC_PUB_TIMEOUT * SYS_TMR_TickCounterFrequencyGet())

static void APP_DebugPrint(uint8_t *pBuf, size_t len)
{
    if ((len > 0) && (len < APP_PRINT_BUFFER_SIZ))
    {
        if (OSAL_RESULT_TRUE == OSAL_MUTEX_Lock(&consoleMutex, OSAL_WAIT_FOREVER))
        {
            if ((len + printBuffPtr) > APP_PRINT_BUFFER_SIZ)
            {
                printBuffPtr = 0;
            }

            memcpy(&printBuff[printBuffPtr], pBuf, len);
            SYS_CONSOLE_Write(0, &printBuff[printBuffPtr], len);

            printBuffPtr = (printBuffPtr + len + 3) & ~3;

            OSAL_MUTEX_Unlock(&consoleMutex);
        }
    }
}

void APP_DebugPrintf(const char* format, ...)
{
    char tmpBuf[APP_PRINT_BUFFER_SIZ];
    size_t len = 0;
    va_list args;
    va_start( args, format );

    len = vsnprintf(tmpBuf, APP_PRINT_BUFFER_SIZ, format, args);

    va_end( args );

    APP_DebugPrint((uint8_t*)tmpBuf, len);
}

char APP_HexToChar(uint8_t hex)
{
    if (hex < 10)
        return '0' + hex;

    if (hex < 16)
        return 'A' + (hex - 10);

    return '-';
}

void APP_DebugPrintBuffer(const uint8_t *pBuf, uint16_t bufLen)
{
    uint8_t tmpBuf[APP_PRINT_BUFFER_SIZ];
    size_t len = 0;
    uint16_t i;
    uint8_t *pB;

    if ((NULL == pBuf) || (0 == bufLen))
        return;

    if (bufLen > (APP_PRINT_BUFFER_SIZ/2))
        bufLen = (APP_PRINT_BUFFER_SIZ/2);

    pB = tmpBuf;
    for (i=0; i<bufLen; i++)
    {
        *pB++ = APP_HexToChar((pBuf[i] & 0xf0) >> 4);
        *pB++ = APP_HexToChar(pBuf[i] & 0x0f);
    }

    len = bufLen*2;

    APP_DebugPrint(tmpBuf, len);
}

void APP_CheckTimeOut(uint32_t timeOutValue, uint32_t lastTimeOut)
{
    char message[128] = {0};
    bool publishMsg = false;
    static bool prevGreenLedState = false;
    static bool prevRedLedState = false;
    bool currGreenLedState = (GREEN_LED_Get()==1) ? true : false;
    bool currRedLedState   = (RED_LED_Get()==1) ? true : false;
    
    if (timeOutValue == 0) {
        return;
    }

    if (SYS_MQTT_STATUS_MQTT_CONNECTED != APP_MQTT_GetStatus(NULL)) {
        return;
    }

    if (lastTimeOut == 0) {
        g_lastPubTimeout = SYS_TMR_TickCountGet();
        return;
    }

    if (SYS_TMR_TickCountGet() - lastTimeOut > timeOutValue)
    {
        static bool led = true;
        
        if( led == 1 )
            sprintf(message, "{\"state\":{\"desired\":{\"greenLED\":\"%s\"}}}", ((currGreenLedState==true) ? "off" : "on"));
        else
            sprintf(message, "{\"state\":{\"desired\":{\"redLED\":\"%s\"}}}", ((currRedLedState==true) ? "off" : "on"));
        
        led = !led;
        g_lastPubTimeout = SYS_TMR_TickCountGet();
        publishMsg = true;
    }
    else if( currGreenLedState != prevGreenLedState )
    {
        sprintf(message, "{\"state\":{\"reported\":{\"greenLED\": \"%s\"}}}", ((currGreenLedState==true) ? "on" : "off"));
        prevGreenLedState = currGreenLedState;
        publishMsg = true;
    }
    else if( currRedLedState != prevRedLedState )
    {
        sprintf(message, "{\"state\":{\"reported\":{\"redLED\": \"%s\"}}}", ((currRedLedState==true) ? "on" : "off"));
        prevRedLedState = currRedLedState;
        publishMsg = true;
    }
    
    if( publishMsg == true )
    {
        if (APP_MQTT_PublishMsg(message) == SYS_MQTT_SUCCESS) 
        {
            SYS_CONSOLE_PRINT("\nPublished Msg(%s) to Topic\r\n", message);
        }
    }
}

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Initialize ( void )
{
    g_netServHandle = SYS_NET_Open(NULL, NULL, 0); 
    if(g_netServHandle != SYS_MODULE_OBJ_INVALID)
        SYS_CONSOLE_PRINT("NET Service Initialized Successfully\r\n");
    
    /* Place the App state machine in its initial state. */
    g_appData.state = APP_STATE_INIT_ECC608;
    
    printBuffPtr = 0;
    OSAL_MUTEX_Create(&consoleMutex);
    kta_gCloudWifiState = CLOUD_STATE_CLOUD_DISCONNECT;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Tasks ( void )
{
    TKStatus ktastatus = E_K_STATUS_ERROR;
    
    SYS_CMD_READY_TO_READ();

    /* Check the application's current state. */
    switch ( g_appData.state )
    {
        case APP_STATE_INIT_ECC608:
        {
            SYS_CONSOLE_MESSAGE(TERM_YELLOW"#########################################\r\n"TERM_RESET);
            SYS_CONSOLE_MESSAGE(TERM_CYAN"  WFI32E03 KeySTREAM Cloud Connect Demo\r\n"TERM_RESET);
            SYS_CONSOLE_MESSAGE(TERM_YELLOW"#########################################\r\n"TERM_RESET);

            // Initialize the ECC608 TrustManager IC
            if( atcab_init(&atecc608_0_init_data) != ATCA_SUCCESS )
            {
                SYS_CONSOLE_MESSAGE(TERM_RED"ECC608 init failed\r\n"TERM_RESET);
                g_appData.state = APP_STATE_ERROR;
                kta_gCloudWifiState = CLOUD_STATE_UNKNOWN;
                break;
            }
            else
            {
                SYS_CONSOLE_MESSAGE(TERM_GREEN"ECC608 init successful\r\n"TERM_RESET);
                g_appData.state = APP_STATE_OPEN_WIFI_DRIVER;
            }
            break;
        }
        case APP_STATE_OPEN_WIFI_DRIVER:
        {
            uint8_t val = SYS_WIFI_GetStatus(sysObj.syswifi);

            if( val == SYS_WIFI_STATUS_TCPIP_READY)
            {
                SYS_CONSOLE_MESSAGE(TERM_GREEN"WIFI UP\r\n\n"TERM_RESET);
                g_appData.state = APP_STATE_INIT_CRYPTOAUTHLIB;
                kta_gCloudWifiState = CLOUD_STATE_WIFI_CONNECTED;
            }
            else
                SYS_CONSOLE_MESSAGE(TERM_RED"=\r"TERM_RESET);
            break;
        }
        case APP_STATE_INIT_CRYPTOAUTHLIB:
        {
            SYS_CONSOLE_MESSAGE(TERM_YELLOW"Initialize KeySTREAM\r\n"TERM_RESET);

            ktastatus = ktaKeyStreamInit();
            if (ktastatus != E_K_STATUS_OK)
            {
                SYS_CONSOLE_MESSAGE(TERM_RED"ktaKeyStreamInit failed\r\n"TERM_RESET);
                g_appData.state = APP_STATE_ERROR;
                kta_gCloudWifiState = CLOUD_STATE_UNKNOWN;
                break;
            }

            g_appData.state = APP_STATE_CONNECT_TO_KEYSTREAM;
            
            break;
        }
        case APP_STATE_CONNECT_TO_KEYSTREAM:
        {
            int delay_counter = 0u;
            SYS_CONSOLE_MESSAGE(TERM_YELLOW"Connect to KeySTREAM\r\n"TERM_RESET);
            
            /* Calling KTA keySTREAM field management. */
            ktastatus = ktaKeyStreamFieldMgmt(true , &ktaKSCmdStatus);
            if (ktastatus != E_K_STATUS_OK)
            {
                SYS_CONSOLE_MESSAGE(TERM_RED"ktaKeyStreamFieldMgmt failed\r\n"TERM_RESET);

                //Over all delay of 30 seconds in case of error.
                while (delay_counter < KEYSTREAM_NO_OPS_DELAY_COUNTER)
                {
                    hal_rtos_delay_ms(KEYSTREAM_NO_OPS_INTERVAL);   // Sleep for 1 second
                    delay_counter++;
                }
                break;
            }

            if (ktaKSCmdStatus == E_K_KTA_KS_STATUS_NO_OPERATION)
            {
                SYS_CONSOLE_MESSAGE(TERM_YELLOW"Device onboarding in keySTREAM done. Device is in ACTIVATED state.\r\n"TERM_RESET);
            }

            /* Checking KTA status for Renewed or Refurbished. */
            if (ktaKSCmdStatus == E_K_KTA_KS_STATUS_RENEW || ktaKSCmdStatus == E_K_KTA_KS_STATUS_REFURBISH)
            {
                SYS_CONSOLE_MESSAGE(TERM_YELLOW"Device is in RENEWED/REFURBISHED state, re-initiate keySTREAM onboarding.\r\n\n"TERM_RESET);
                g_appData.state = APP_STATE_OPEN_WIFI_DRIVER;
            }
            else if (kta_gCloudWifiState != CLOUD_STATE_CLOUD_CONNECTED)
            {
                if (kta_gCloudWifiState != CLOUD_STATE_CLOUD_DISCONNECT)
                {
                    SYS_CONSOLE_MESSAGE(TERM_YELLOW"Finished KeySTREAM\r\n\n"TERM_RESET);
                    kta_gCloudWifiState = CLOUD_STATE_CLOUD_CONNECTING;
                    SYS_CONSOLE_MESSAGE("Device is now connecting to the cloud provider.\r\n");
                    g_appData.state = APP_STATE_SERVICE_TASKS;
                }
            }

            break;
        }
        // Perform service tasks        
        case APP_STATE_SERVICE_TASKS:
        {
            if( kta_gCloudWifiState == CLOUD_STATE_CLOUD_CONNECTING )
            {
                SYS_CONSOLE_PRINT("Resolving cloud hostname %s\r\n\n", SYS_MQTT_INDEX0_BROKER_NAME);

                APP_MQTT_Initialize();
                kta_gCloudWifiState = CLOUD_STATE_CLOUD_CONNECT;
            }
            else
            {
                SYS_CONSOLE_MESSAGE(TERM_GREEN"=\r"TERM_RESET);
                APP_CheckTimeOut(MQTT_PUB_TIMEOUT_CONST, g_lastPubTimeout);
                
                APP_MQTT_Tasks();
            }
            break;
        }
        case APP_STATE_CLOUD_CONNECTED:
        {
            APP_MQTT_Subscribe();
            
            g_appData.state = APP_STATE_SERVICE_TASKS;
            kta_gCloudWifiState = CLOUD_STATE_CLOUD_CONNECTED;
            break;
        }
        case APP_STATE_CLOUD_SUBSCRIBED:
        {
            char message[128] = {0};
            strcpy(message, "{\"state\":{\"reported\":{\"greenLED\": \"off\",\"redLED\":\"off\"},\"desired\":{\"greenLED\": \"off\",\"redLED\":\"off\"}}}");
			APP_MQTT_PublishMsg(message);
            
            g_appData.state = APP_STATE_SERVICE_TASKS;
            break;
        }
        default:
            break;
    }
}


/*******************************************************************************
 End of File
 */
