/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony MQTT application.

  Description:
    This file contains the source code for the MPLAB Harmony MQTT application.
    It implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/
/*******************************************************************************
Copyright (C) 2021 released Microchip Technology Inc.  All rights reserved.

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

#include "app_mqtt.h"
#include "system/mqtt/sys_mqtt.h"
#include "cloud_wifi_ecc_process.h"
#include "cloud_wifi_config.h"
#include "jwt/atca_jwt.h"
#include <stdio.h>
#include "definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Declarations
// *****************************************************************************
// *****************************************************************************
//int APP_MQTT_GetClientPubTopic(char* buf, size_t buflen);

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
extern APP_DATA g_appData;
extern uint8_t subject_key_id[20];
SYS_MODULE_OBJ g_sSysMqttHandle = SYS_MODULE_OBJ_INVALID;
SYS_MQTT_Config g_sTmpSysMqttCfg;

#define SYS_MQTT_DEF_PUB_RETAIN     0
#define SYS_MQTT_DEF_PUB_QOS        1

//#define APP_CFG_WITH_MQTT_API

// *****************************************************************************
// *****************************************************************************
// Section: Local data
// *****************************************************************************
// *****************************************************************************

int32_t APP_MQTT_PublishMsg(char *message)
{
	SYS_MQTT_PublishTopicCfg sMqttTopicCfg;
	int32_t retVal = SYS_MQTT_FAILURE;

	APP_MQTT_GetClientPubTopic(sMqttTopicCfg.topicName, 100);
    sMqttTopicCfg.topicLength = strlen(sMqttTopicCfg.topicName);
	sMqttTopicCfg.retain = SYS_MQTT_DEF_PUB_RETAIN;
	sMqttTopicCfg.qos = SYS_MQTT_DEF_PUB_QOS;

	retVal = SYS_MQTT_Publish(g_sSysMqttHandle,	&sMqttTopicCfg, message, strlen(message));
	if (retVal != SYS_MQTT_SUCCESS) {
		SYS_CONSOLE_PRINT("\nPublish_PeriodicMsg(): Failed (%d)\r\n", retVal);
	}
	return retVal;
}

int32_t APP_MQTT_Subscribe(void)
{
    SYS_MQTT_SubscribeConfig sMqttSubCfg;
	int32_t retVal = SYS_MQTT_FAILURE;
    
    APP_MQTT_GetClientSubTopic(sMqttSubCfg.topicName, 100);
	sMqttSubCfg.entryValid = 0;
	sMqttSubCfg.qos = SYS_MQTT_DEF_PUB_QOS;
    
	SYS_CONSOLE_PRINT("\nSubscribeTopic: %s\r\n", sMqttSubCfg.topicName);

    retVal = SYS_MQTT_Subscribe(g_sSysMqttHandle, &sMqttSubCfg);
    if (retVal != SYS_MQTT_SUCCESS)
    {
		SYS_CONSOLE_PRINT("\nSubscribe to Topic failed (%d)\r\n", retVal);
    }
 	return retVal;   
}

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    int32_t MqttCallback(SYS_MQTT_EVENT_TYPE eEventType, void *data, uint16_t len, void* cookie)

  Remarks:
    Callback function registered with the SYS_MQTT_Connect() API. For more details 
	check https://microchip-mplab-harmony.github.io/wireless/system/mqtt/docs/interface.html
 */
int32_t MqttCallback(SYS_MQTT_EVENT_TYPE eEventType, void *data, uint16_t len, void* cookie)
{       
    switch (eEventType)
    {
        case SYS_MQTT_EVENT_MSG_RCVD:
        {
			/* Message received on Subscribed Topic */
            SYS_MQTT_PublishConfig *psMsg = (SYS_MQTT_PublishConfig*)data;
            psMsg->message[psMsg->messageLength] = 0;
            psMsg->topicName[psMsg->topicLength] = 0;

            SYS_CONSOLE_PRINT("\nMqttCallback():\r\nMsg received on Topic: %s\r\nMsg: %s\r\n", psMsg->topicName, psMsg->message);

            char *pPtr;
            pPtr = strstr(psMsg->message, "redLED");
            if (pPtr != NULL)
            {
                RED_LED_Toggle();
            }

            pPtr = strstr(psMsg->message, "greenLED");
            if (pPtr != NULL)
            {
                GREEN_LED_Toggle();
            }
            break;
        }
        case SYS_MQTT_EVENT_MSG_DISCONNECTED:
        {
            break;
        }
        case SYS_MQTT_EVENT_MSG_CONNECTED:
        {
			SYS_CONSOLE_PRINT("\nMqttCallback(): Connected\r\n");
            g_appData.state = APP_STATE_CLOUD_CONNECTED;
            break;
        }
        case SYS_MQTT_EVENT_MSG_SUBSCRIBED:
        {
            SYS_MQTT_SubscribeConfig *psMqttSubCfg = (SYS_MQTT_SubscribeConfig	*)data;
            SYS_CONSOLE_PRINT("\nMqttCallback(): Subscribed to Topic '%s'\r\n", psMqttSubCfg->topicName);
            g_appData.state = APP_STATE_CLOUD_SUBSCRIBED;
            break;
        }
        case SYS_MQTT_EVENT_MSG_UNSUBSCRIBED:
        {
			/* MQTT Topic Unsubscribed; Now the Client will not receive any messages for this Topic */
            break;
        }
        case SYS_MQTT_EVENT_MSG_PUBLISHED:
        {
			/* MQTT Client Msg Published */
            SYS_CONSOLE_PRINT("\nMqttCallback(): Published\r\n");
            break;
        }
        case SYS_MQTT_EVENT_MSG_CONNACK_TO:
        {
			/* MQTT Client ConnAck TimeOut; User will need to reconnect again */
            break;
        }
        case SYS_MQTT_EVENT_MSG_SUBACK_TO:
        {
			/* MQTT Client SubAck TimeOut; User will need to subscribe again */
            break;
        }
        case SYS_MQTT_EVENT_MSG_PUBACK_TO:
        {
			/* MQTT Client PubAck TimeOut; User will need to publish again */
            break;
        }
        case SYS_MQTT_EVENT_MSG_UNSUBACK_TO:
        {
			/* MQTT Client UnSubAck TimeOut; User will need to Unsubscribe again */
            break;
        }
        case SYS_MQTT_EVENT_MSG_SOCK_CONNECTED:
        {
            /* socket connection has been establish */
            SYS_MQTT_BrokerConfig *psMsg = (SYS_MQTT_BrokerConfig*)data;

            APP_MQTT_GetClientId(psMsg->clientId, sizeof(subject_key_id)*2+1);
            SYS_CONSOLE_PRINT("\nMqttCallback(): Socket Connected, ID=%s\r\n", psMsg->clientId);
            break;
        }
    }
    return SYS_MQTT_SUCCESS;
}

/*******************************************************************************
  Function:
    void APP_MQTT_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_MQTT_Initialize(void) {

	/*
	** For more details check https://microchip-mplab-harmony.github.io/wireless/system/mqtt/docs/interface.html
	*/
#ifdef APP_CFG_WITH_MQTT_API

	/* In case the user does not want to use the configuration given in the MHC */
    SYS_MQTT_Config *psMqttCfg;

    memset(&g_sTmpSysMqttCfg, 0, sizeof (g_sTmpSysMqttCfg));
    psMqttCfg = &g_sTmpSysMqttCfg;
    psMqttCfg->sBrokerConfig.autoConnect = true;
    psMqttCfg->sBrokerConfig.tlsEnabled = true;
    strcpy(psMqttCfg->sBrokerConfig.brokerName, SYS_MQTT_INDEX0_BROKER_NAME);
    psMqttCfg->sBrokerConfig.serverPort = SYS_MQTT_INDEX0_MQTT_PORT;
    psMqttCfg->sBrokerConfig.cleanSession = true;
    psMqttCfg->sBrokerConfig.keepAliveInterval = 60;
    psMqttCfg->subscribeCount = 0;
    APP_MQTT_GetClientId(psMqttCfg->sBrokerConfig.clientId, sizeof(subject_key_id)*2+1);
    g_sSysMqttHandle = SYS_MQTT_Connect(&g_sTmpSysMqttCfg, MqttCallback, NULL);
#else    
    g_sSysMqttHandle = SYS_MQTT_Connect(NULL, /* NULL value means that the MHC configuration should be used for this connection */
										MqttCallback, 
										NULL);
#endif    
}

/******************************************************************************
  Function:
    void APP_MQTT_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_MQTT_Tasks(void) {

    SYS_MQTT_Task(g_sSysMqttHandle);
}

/******************************************************************************
  Function:
    int32_t APP_MQTT_GetStatus ( void *)

  Remarks:
    See prototype in app.h.
 */
int32_t APP_MQTT_GetStatus(void *p) {

    return SYS_MQTT_GetStatus(g_sSysMqttHandle);
}



static char config_thing_id[130];

int APP_MQTT_SetThingId()
{
    uint8_t serial_num[9];
    size_t hex_size;

    hex_size = sizeof(config_thing_id) - 1;
    ATCA_STATUS rv;

    rv = atcab_read_serial_number(serial_num);
    if (ATCA_SUCCESS != rv)
    {
        return rv;
    }
    rv = atcab_bin2hex_(serial_num, sizeof(serial_num), config_thing_id, &hex_size, false, false, true);

    if (ATCA_SUCCESS != rv)
    {
        return rv;
    }

    return ATCA_SUCCESS;
}

/** \brief Populate the buffer with the client id */
int APP_MQTT_GetClientId(char* buf, size_t buflen)
{
    if (buf && buflen)
    {
        int rv;

#if defined(CLOUD_CONFIG_GCP)
        rv = snprintf(buf, buflen, "projects/%s/locations/%s/registries/%s/devices/d%s",
                      config_gcp_project_id, config_gcp_region_id, config_gcp_registry_id, config_thing_id);
#elif defined(CLOUD_CONFIG_AWS)
    #ifdef CLOUD_CONNECT_WITH_CUSTOM_CERTS

        for (int i = 0; i < 20; i++)
        {
            config_thing_id[i * 2 + 0] = "0123456789abcdef"[subject_key_id[i] >> 4];
            config_thing_id[i * 2 + 1] = "0123456789abcdef"[subject_key_id[i] & 0x0F];
        }
        config_thing_id[20 * 2] = 0; // Add terminating null
    #endif
        rv = snprintf(buf, buflen, "%s", config_thing_id);
#elif defined(CLOUD_CONFIG_AZURE)
        rv = snprintf(buf, buflen, "sn%s", config_thing_id);
#else
    #error "Cloud config should be defined"
#endif

        if (0 < rv && rv < buflen)
        {
            buf[rv] = 0;
            return 0;
        }
    }
    return -1;
}

/* Populate the buffer with the username */
int APP_MQTT_GetClientUsername(char* buf, size_t buflen)
{
    if (buf && buflen)
    {
#ifdef CLOUD_CONFIG_AZURE
        int rv = snprintf(buf, buflen, "%s/sn%s", CLOUD_ENDPOINT, config_thing_id);
#else
        int rv = snprintf(buf, buflen, "unused");
#endif
        if (0 < rv && rv < buflen)
        {
            buf[rv] = 0;
            return 0;
        }
    }
    return -1;
}

/* Populate the buffer with the user's password */
int APP_MQTT_GetClientPassword(char* buf, size_t buflen)
{
    int rv = -1;

#ifdef CLOUD_CONFIG_GCP

    if (buf && buflen)
    {
        atca_jwt_t jwt;

        uint32_t ts = RTC_Timer32CounterGet();

        /* Build the JWT */
        rv = atca_jwt_init(&jwt, buf, buflen);
        if (ATCA_SUCCESS != rv)
        {
            return rv;
        }

        if (ATCA_SUCCESS != (rv = atca_jwt_add_claim_numeric(&jwt, "iat", ts)))
        {
            return rv;
        }

        if (ATCA_SUCCESS != (rv = atca_jwt_add_claim_numeric(&jwt, "exp", ts + 86400)))
        {
            return rv;
        }

        if (ATCA_SUCCESS != (rv = atca_jwt_add_claim_string(&jwt, "aud", config_gcp_project_id)))
        {
            return rv;
        }

        rv = atca_jwt_finalize(&jwt, 0);

    }
#else
    if (buf && buflen)
    {
        rv = snprintf(buf, buflen, "unused");

        if (0 < rv && rv < buflen)
        {
            buf[rv] = 0;
            return 0;
        }
    }
#endif

    return rv;

}

/* Get the topic id  where the client will be publishing messages */
int APP_MQTT_GetClientSubTopic(char* buf, size_t buflen)
{
    int rv;

    if (buf && buflen)
    {
#if defined(CLOUD_CONFIG_GCP)
        rv = snprintf(buf, buflen, "/devices/d%s/events", config_thing_id);
#elif defined(CLOUD_CONFIG_AWS)
        rv = snprintf(buf, buflen, "$aws/things/%s/shadow/update/delta", config_thing_id);
#elif defined(CLOUD_CONFIG_AZURE)
        rv = snprintf(buf, buflen, "devices/sn%s/messages/events/", config_thing_id);
#endif

        if (0 < rv && rv < buflen)
        {
            buf[rv] = 0;
            return 0;
        }
    }
    return -1;
}

/* Get the topic id  where the client will be publishing messages */
int APP_MQTT_GetClientPubTopic(char* buf, size_t buflen)
{
    int rv;

    if (buf && buflen)
    {

#if defined(CLOUD_CONFIG_GCP)
        rv = snprintf(buf, buflen, "/devices/d%s/commands/#", config_thing_id);
#elif defined(CLOUD_CONFIG_AWS)
        rv = snprintf(buf, buflen, "$aws/things/%s/shadow/update", config_thing_id);
#elif defined(CLOUD_CONFIG_AZURE)
        rv = snprintf(buf, buflen, "devices/sn%s/messages/devicebound/#", config_thing_id);
#endif

        if (0 < rv && rv < buflen)
        {
            buf[rv] = 0;
            return 0;
        }
    }
    return -1;
}

/*******************************************************************************
 End of File
 */
