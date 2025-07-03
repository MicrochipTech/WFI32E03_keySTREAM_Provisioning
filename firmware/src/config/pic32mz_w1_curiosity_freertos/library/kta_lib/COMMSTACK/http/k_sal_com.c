// Uses NET_PRES layer
#if 1


/*******************************************************************************
*************************keySTREAM Trusted Agent ("KTA")************************

* (c) 2023-2024 Nagravision Sàrl

* Subject to your compliance with these terms, you may use the Nagravision Sàrl
* Software and any derivatives exclusively with Nagravision's products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may accompany
* Nagravision Software.

* Redistribution of this Nagravision Software in source or binary form is allowed
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.

* THIS SOFTWARE IS SUPPLIED BY NAGRAVISION "AS IS". NO WARRANTIES, WHETHER EXPRESS,
* IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
* NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN NO
* EVENT WILL NAGRAVISION BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL
* OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF NAGRAVISION HAS BEEN ADVISED OF THE
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW,
* NAGRAVISION 'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
* SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY
* TO NAGRAVISION FOR THIS SOFTWARE.
********************************************************************************/
/** \brief  Interface for socket communication.
 *
 *  \author Kudelski IoT
 *
 *  \date 2023/06/12
 *
 *  \file k_sal_com.c
 ******************************************************************************/
/**
 * @brief Interface for socket communication.
 */

#include "k_sal_com.h"
/* -------------------------------------------------------------------------- */
/* IMPORTS                                                                    */
/* -------------------------------------------------------------------------- */
#include "net_pres/pres/net_pres_socketapi.h"
#include "system/debug/sys_debug.h"
// #include "dns.h"
#include "cryptoauthlib.h"

#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* LOCAL CONSTANTS, TYPES, ENUM                                               */
/* -------------------------------------------------------------------------- */

/** @brief Invalid Socket Id. */
#define C_SAL_COM_SOCKET_INVALID (-1)

/** @brief Connect wait time in ms. */
#define C_SAL_COM_CONNECT_DELAY_IN_MS       (500u)

/**
 * @brief
 * Reset read time out value in ms(this will override the read timeout
 * after first successful read. This is added to avoid more dealy in read because
 * read will always hit the timeout while checking any bytes are available or not.
 */
#define C_SAL_COM_RESET_READ_TIMEOUT_IN_MS  (100u)

/** @brief Length of an address in IP V6 protocol, in bytes. */
#define C_SAL_COM_V6_ADDRESS_NUM_BYTES      (16u)

#define C_SAL_COM_MAX_READ_RETRY_COUNT      (10u)

/******************************************************************************/
/* LOCAL MACROS                                                               */
/******************************************************************************/

#ifdef DEBUG
/** @brief Enable sal com logs. */
#define M_INTL_SAL_COM_DEBUG(__PRINT__)  do { \
                                              SYS_CONSOLE_PRINT("\tCOM %d>", __LINE__); \
                                              SYS_CONSOLE_PRINT __PRINT__; \
                                              SYS_CONSOLE_PRINT("\r\n"); \
                                          } while (0)
#define M_INTL_SAL_COM_ERROR(__PRINT__)  do { \
                                              SYS_CONSOLE_PRINT("\tCOM %d> ERROR ", __LINE__); \
                                              SYS_CONSOLE_PRINT __PRINT__; \
                                              SYS_CONSOLE_PRINT("\r\n"); \
                                          } while (0)
#else
#define M_INTL_SAL_COM_DEBUG(__PRINT__)
#define M_INTL_SAL_COM_ERROR(__PRINT__)
#endif /* DEBUG. */

/******************************************************************************/
/* TYPES & STRUCTURES                                                         */
/******************************************************************************/

/** @brief Boolean Types. */
typedef enum
{
  E_FALSE,
  /* TRUE. */
  E_TRUE,
  /* FALSE. */
  E_UNDEF
  /* Undefined. */
} TBoolean;

/**
 * @ingroup g_k_types
 *
 * @brief Supported IP protocols.
 */
typedef enum
{
  E_K_IP_PROTOCOL_V4,
  /* IP V4. */
  E_K_IP_PROTOCOL_V6,
  /* IP V6. */
  E_K_IP_NUM_PROTOCOLS
  /* Number of supported IP protocols. */
} TKIpProtocol;

/**
 * @ingroup g_k_types
 *
 * @brief Socket IP address, expressed for protocol IP V4.
*/
typedef struct
{
  uint32_t       address;
  /* IP V4 address, MSBF; e.g. 192.168.15.4 = 0xC0A80F04 */
  uint16_t       port;
  /* IP port. */
} TKSocketAddressV4;

/**
 * @ingroup g_k_types
 *
 * @brief Socket IP.
 */
typedef struct
{
  TKIpProtocol         protocol;
  /* IP protocol. */
  TKSocketAddressV4    v4;
  /* IP address; the used field depends on protocol value. */
} TKSocketIp;

/**
 * @ingroup g_k_types
 *
 * @brief Communication info.
 */
typedef struct
{
  int       socketId;
  /* ID of unique socket used. */
  uint32_t  connectTimeOut;
  /* Connection timeout. */
  uint32_t  readTimeOut;
  /* Read timeout. */
  uint32_t  isConnected;
  /* Flag to check whether already connection estabilished with server or not. */
} TKComInfo;

/* -------------------------------------------------------------------------- */
/* LOCAL VARIABLES                                                            */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* LOCAL FUNCTIONS - PROTOTYPE                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief
 *   Convert the ip to sal socket structure.
 *
 * @param[in] xpAddress
 *   Ip address of the server. Should not be NULL
 * @param[out] xpSocketIp
 *   Converted SAL socket ip.
 * @return
 * - E_TRUE for success
 * - E_FALSE for failure
 */
#if 0
static TBoolean lConvertSocketIp
(
  const uint8_t*  xpAddress,
  TKSocketIp*     xpSocketIp
);

#endif

/* -------------------------------------------------------------------------- */
/* PUBLIC VARIABLES                                                           */
/* -------------------------------------------------------------------------- */

extern int32_t kta_gRxBufferLength;
extern enum wifi_status kta_gSockRecvStatus;
extern enum wifi_status kta_gSockSendStatus;
extern uint32_t kta_gTxSize;
extern uint8_t* kta_getHostByName(const char* xpUrl);

/* -------------------------------------------------------------------------- */
/* PUBLIC FUNCTIONS - IMPLEMENTATION                                          */
/* -------------------------------------------------------------------------- */

// TODO: Clean up naming of entire module
// TODO: Add function declarations, descriptions, etc.

// Global variables to track the last RX and TX times
static uint32_t lastRxTime = 0;
static uint32_t lastTxTime = 0;

void RxSignalHandler(NET_PRES_SKT_HANDLE_T hNet, NET_PRES_SIGNAL_HANDLE hSig, uint16_t sigType, const void* param) {
    // Update the last RX time
    lastRxTime = SYS_TMR_TickCountGet();
}

void TxSignalHandler(NET_PRES_SKT_HANDLE_T hNet, NET_PRES_SIGNAL_HANDLE hSig, uint16_t sigType, const void* param) {
    // Update the last TX time
    lastTxTime = SYS_TMR_TickCountGet();
}


TKCommStatus GetIpFromHost(const uint8_t* dnsLookupTargetHost, IP_MULTI_ADDRESS* ipAddr)
{
  TCPIP_DNS_RESULT dnsResult = TCPIP_DNS_RES_PENDING;
  TKCommStatus status = E_K_COMM_STATUS_ERROR;
  uint32_t dnsLookUpStartTick = SYS_TMR_TickCountGet();
  uint32_t timeout = 0;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));

  for (;;)
  {
    if ((NULL == dnsLookupTargetHost) || (NULL == ipAddr))
    {
      M_INTL_SAL_COM_ERROR(("Invalid parameter"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }

    M_INTL_SAL_COM_DEBUG(("Resolving hostname %s", dnsLookupTargetHost));
    TCPIP_DNS_Resolve((const char *)dnsLookupTargetHost, TCPIP_DNS_TYPE_A);
    // TODO: Retries?
    while(TCPIP_DNS_RES_PENDING == dnsResult)
    {
      timeout = (SYS_TMR_TickCountGet() - dnsLookUpStartTick) / SYS_TMR_TickCounterFrequencyGet();
      // TODO: #include configuration.h (for TCPIP_DNS_CLIENT_SERVER_TMO)
      if(timeout >= (TCPIP_DNS_CLIENT_SERVER_TMO / 2))
      {
        M_INTL_SAL_COM_ERROR(("DNS Lookup: request timeout"));
        break;
      }

      atca_delay_ms(5);
      dnsResult = TCPIP_DNS_IsResolved((const char *)dnsLookupTargetHost, ipAddr, TCPIP_DNS_TYPE_A);
    }

    if ((TCPIP_DNS_RES_OK == dnsResult) || (TCPIP_DNS_RES_NAME_IS_IPADDRESS == dnsResult))
    {
    //   M_INTL_SAL_COM_DEBUG(("Resolved hostname %s as IP %u.%u.%u.%u",
    //                         (const char *)dnsLookupTargetHost,
    //                         ipAddr->v4Add.v[0],
    //                         ipAddr->v4Add.v[1],
    //                         ipAddr->v4Add.v[2],
    //                         ipAddr->v4Add.v[3]));
      status = E_K_COMM_STATUS_OK;
    }
    else
    {
      M_INTL_SAL_COM_ERROR(("DNS Lookup failed with status %d", dnsResult));
    }

    break;
  }

  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));

  return status;
}

TKCommStatus WaitForConnection(void* xpComInfo)
{
  TKComInfo* pComInfo = (TKComInfo *)xpComInfo;
  TKCommStatus status = E_K_COMM_STATUS_OK;
  uint32_t connectStartTick = SYS_TMR_TickCountGet();
  uint32_t timeout = 0;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));

  for (;;)
  {
    if (NULL == xpComInfo)
    {
      M_INTL_SAL_COM_ERROR(("Invalid parameter"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }

    while (false == NET_PRES_SocketIsConnected(pComInfo->socketId))
    {
      timeout = (SYS_TMR_TickCountGet() - connectStartTick) * 1000 / SYS_TMR_TickCounterFrequencyGet();
      M_INTL_SAL_COM_DEBUG(("waiting: %d vs %d", timeout, pComInfo->connectTimeOut / 2));
      if(timeout >= (pComInfo->connectTimeOut / 2))
      {
        M_INTL_SAL_COM_ERROR(("Socket connection timed out."));
        status = E_K_COMM_STATUS_ERROR;
        break;
      }

      atca_delay_ms(5);
    }
    break;
  }

  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));

  return status;
}

NET_PRES_SIGNAL_HANDLE sigHandler;

static void _socketSignalHandler(NET_PRES_SKT_HANDLE_T handle, NET_PRES_SIGNAL_HANDLE hNet, uint16_t sigType, const void* param) 
{
    M_INTL_SAL_COM_DEBUG(("_socketSignalHandler sigType %x\n", sigType));
}

TKCommStatus CreateSocket(void* xpComInfo, const uint8_t* xpHost, const uint8_t* xpPort)
{
  TKComInfo*       pComInfo = (TKComInfo *)xpComInfo;
  TKCommStatus     status   = E_K_COMM_STATUS_OK;
  IP_MULTI_ADDRESS addr     = { 0 };
  uint16_t         port     = 0;
  char*            pEnd     = NULL;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));

  for (;;)
  {
    if (NULL == xpPort)
    {
      M_INTL_SAL_COM_ERROR(("Invalid parameter"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }

    errno = 0;
    port = (uint16_t)strtol((const char *)xpPort, &pEnd, 10);
    if (0 != errno)
    {
      M_INTL_SAL_COM_ERROR(("Error occurred %d", errno));
      break;
    }

    if (NET_PRES_INVALID_SOCKET == pComInfo->socketId)
    {
      if (E_K_COMM_STATUS_OK != GetIpFromHost(xpHost, &addr))
      {
        M_INTL_SAL_COM_ERROR(("Failed to resolve URL to ip address"));
        status = E_K_COMM_STATUS_PARAMETER;
        break;
      }

      if (0 == addr.v4Add.Val)
      {
        M_INTL_SAL_COM_ERROR(("Got invalid ip address"));
        status = E_K_COMM_STATUS_RESOURCE;
        break;
      }
      M_INTL_SAL_COM_DEBUG(("Open socket at %u.%u.%u.%u:%u / %u",
                            addr.v4Add.v[0],
                            addr.v4Add.v[1],
                            addr.v4Add.v[2],
                            addr.v4Add.v[3],
                            port,
                            addr.v4Add.Val));

      static NET_PRES_SKT_ERROR_T sockError;
      pComInfo->socketId = NET_PRES_SocketOpen(0, NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT, IP_ADDRESS_TYPE_IPV4, port, (NET_PRES_ADDRESS*)&addr, &sockError);
      // pComInfo->socketId = NET_PRES_SocketOpen(0, NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT, IP_ADDRESS_TYPE_ANY, port, (NET_PRES_ADDRESS*)&addr, &sockError);

      if (NET_PRES_INVALID_SOCKET == pComInfo->socketId)
      {
        M_INTL_SAL_COM_ERROR(("Invalid socket ID"));
        status = E_K_COMM_STATUS_RESOURCE;
        break;
      }

      M_INTL_SAL_COM_DEBUG(("Socket created successfully with handle %d, error %d", pComInfo->socketId, sockError));
    //   M_INTL_SAL_COM_DEBUG(("Socket created successfully with handle %d, error %d", pComInfo->socketId, 0));

      // TODO: Use NET_PRES_SocketSignalHandlerRegister() to process TCPIP events?
      // Rx and Tx timeouts are not yet implemented by the TCP/IP-stack.
      // So the following two calls to NET_PRES_SocketOptionsSet() will fail for now.

#if 0   
      if (false == NET_PRES_SocketOptionsSet(pComInfo->socketId, TCP_OPTION_TX_TMO, (void*)pComInfo->connectTimeOut))
      {
        M_INTL_SAL_COM_ERROR(("Failed to set socket write timeout"));
      }
      if (false == NET_PRES_SocketOptionsSet(pComInfo->socketId, TCP_OPTION_RX_TMO, (void*)pComInfo->readTimeOut))
      {
        M_INTL_SAL_COM_ERROR(("Failed to set socket read timeout"));
      }
#else
    // Register RX and TX signal handlers
    sigHandler = NET_PRES_SocketSignalHandlerRegister(pComInfo->socketId, TCPIP_TCP_SIGNAL_RX_RST | TCPIP_TCP_SIGNAL_KEEP_ALIVE_TMO, _socketSignalHandler, NULL);      
    sigHandler = NET_PRES_SocketSignalHandlerRegister(pComInfo->socketId, TCPIP_TCP_SIGNAL_TX_RST | TCPIP_TCP_SIGNAL_KEEP_ALIVE_TMO, _socketSignalHandler, NULL);
#endif      
    }
    break;
  }

  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));

  return status;
}

/**
 * @brief  implement salComInit
 *
 */
K_SAL_API TKCommStatus salComInit
(
  uint32_t xConnectTimeoutInMs,
  uint32_t xReadTimeoutInMs,
  void**   xppComInfo
)
{
  static TKComInfo gComInfo = { 0 };
  TKCommStatus     status   = E_K_COMM_STATUS_ERROR;
  TKComInfo*       pComInfo = &gComInfo;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));

  for (;;)
  {
    if (NULL == xppComInfo)
    {
      M_INTL_SAL_COM_ERROR(("Invalid parameter"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }

    pComInfo->socketId     = NET_PRES_INVALID_SOCKET;
    pComInfo->connectTimeOut = xConnectTimeoutInMs;
    pComInfo->readTimeOut  = xReadTimeoutInMs;
    pComInfo->isConnected  = 0;

    *xppComInfo = pComInfo;
    status = E_K_COMM_STATUS_OK;
    break;
  }

  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));
  return status;
}

/**
 * @brief  implement salComConnect
 *
 */
 // TODO: Change type of xpComInfo to TKComInfo? for CreateSocket and WaitForConnection?
K_SAL_API TKCommStatus salComConnect
(
  void*          xpComInfo,
  const uint8_t* xpHost,
  const uint8_t* xpPort
)
{
  TKComInfo*   pComInfo = (TKComInfo *)xpComInfo;
  TKCommStatus status = E_K_COMM_STATUS_ERROR;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));

  for (;;)
  {
    if (E_K_COMM_STATUS_OK != CreateSocket(xpComInfo, xpHost, xpPort))
    {
      M_INTL_SAL_COM_ERROR(("Failed to create socket"));
      status = E_K_COMM_STATUS_ERROR;
      break;
    }

    if (E_K_COMM_STATUS_OK != WaitForConnection(xpComInfo))
    {
      M_INTL_SAL_COM_ERROR(("Connect to server failed"));
      status = E_K_COMM_STATUS_RESOURCE;
      NET_PRES_SocketClose(pComInfo->socketId);
      pComInfo->socketId = INVALID_SOCKET;
      break;
    }

    status = E_K_COMM_STATUS_OK;
    break;
  }

  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));

  return status;
}

/**
 * @brief  implement salComWrite
 *
 */
K_SAL_API TKCommStatus salComWrite
(
  void*          xpComInfo,
  const uint8_t* xpBuffer,
  size_t         xBufferLen
)
{
  TKComInfo*   pComInfo = (TKComInfo *)xpComInfo;
  TKCommStatus status = E_K_COMM_STATUS_ERROR;
  NET_PRES_SKT_ERROR_T sendStatus = NET_PRES_SKT_OK;
  uint32_t overallBytesSent = 0;
  uint32_t bytesSent = 0;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));

  for (;;)
  {
    if ((NULL == xpComInfo) || (NULL == xpBuffer) || (0U == xBufferLen))
    {
      M_INTL_SAL_COM_ERROR(("Invalid parameter"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }

    // Reset last error
    NET_PRES_SocketLastError(pComInfo->socketId);

    while (overallBytesSent < xBufferLen)
    {
      // TODO: Is calling NET_PRES_SocketIsConnected() here necessary/useful?
      if (0 == NET_PRES_SocketIsConnected(pComInfo->socketId))
      {
        M_INTL_SAL_COM_DEBUG(("Socket not connected"));
        break;
      }

      vTaskDelay(5U / portTICK_PERIOD_MS); 
      //atca_delay_ms(5);
      
      bytesSent = NET_PRES_SocketWrite(pComInfo->socketId, &xpBuffer[overallBytesSent], xBufferLen);
      
      if (0 == bytesSent)
      {
        sendStatus = NET_PRES_SocketLastError(pComInfo->socketId);
        if (NET_PRES_SKT_OK != sendStatus)
        {
          M_INTL_SAL_COM_ERROR(("Send failed with status[%d]", sendStatus));
          break;
        }
        else
        {
          // TODO: Flush here to clear the buffer?
          vTaskDelay(1U / portTICK_PERIOD_MS); 
          //atca_delay_ms(1);
        }
      }
      overallBytesSent += bytesSent;
    }

    if (overallBytesSent == xBufferLen)
    {
      status = E_K_COMM_STATUS_OK;
    }
    else
    {
      M_INTL_SAL_COM_ERROR(("Sending failed"));
    }

    break;
  }

  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));

  return status;
}

/**
 * @brief  implement salComRead
 *
 */
K_SAL_API TKCommStatus salComRead
(
  void*     xpComInfo,
  uint8_t*  xpBuffer,
  size_t*   xpBufferLen
)
{
  TKComInfo*   pComInfo = (TKComInfo *)xpComInfo;
  TKCommStatus status = E_K_COMM_STATUS_ERROR;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));

  for (;;)
  {
    if ((NULL == xpComInfo) || (NULL == xpBuffer) || (NULL == xpBufferLen) || (0 == *xpBufferLen))
    {
      M_INTL_SAL_COM_ERROR(("Invalid parameter"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }

    if (0 == NET_PRES_SocketIsConnected(pComInfo->socketId))
    {
      M_INTL_SAL_COM_DEBUG(("Socket not connected"));
      break;
    }

    uint16_t readDataLength = 0;
    uint16_t retryCount;

    // [AB] Manual wait / timeout
    vTaskDelay(1000U / portTICK_PERIOD_MS); 
    //vTaskDelay(2U / portTICK_PERIOD_MS); 

    // TODO: Error handling
    for (retryCount = 0; retryCount < (uint16_t)C_SAL_COM_MAX_READ_RETRY_COUNT; retryCount++)
    {
      readDataLength = NET_PRES_SocketRead(pComInfo->socketId, xpBuffer, *xpBufferLen);
      if (readDataLength != 0)
      {
        // TODO: Ensure readDataLength is <= *xpBufferLen?
        status = E_K_COMM_STATUS_OK;
        *xpBufferLen = (size_t)readDataLength;
        M_INTL_SAL_COM_DEBUG(("readDataLength = %d", readDataLength));
        break;
      }
      else
      {
        M_INTL_SAL_COM_ERROR(("SOCKETS_Recv timed out: %d, %d,retrying", NET_PRES_SocketLastError(pComInfo->socketId), retryCount));
    //    atca_delay_ms(pComInfo->readTimeOut);
        // [AB] Manual wait / timeout
        vTaskDelay(pComInfo->readTimeOut / portTICK_PERIOD_MS); 
      }
    }

    if (retryCount == (uint16_t)C_SAL_COM_MAX_READ_RETRY_COUNT)
    {
      M_INTL_SAL_COM_ERROR(("SOCKETS_Recv number of Timeout exceeded"));
    }

    break;
  }

  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));

  return status;
}

/**
 * @brief  implement salComTerm
 *
 */
K_SAL_API TKCommStatus salComTerm
(
  void*  xpComInfo
)
{
  TKComInfo*   pComInfo = (TKComInfo *)xpComInfo;
  TKCommStatus status = E_K_COMM_STATUS_OK;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));

  for (;;)
  {
    if (NULL == xpComInfo)
    {
      M_INTL_SAL_COM_ERROR(("Invalid parameter"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }

    if (INVALID_SOCKET != pComInfo->socketId)
    {
      NET_PRES_SocketClose(pComInfo->socketId);
      pComInfo->socketId = INVALID_SOCKET;
      pComInfo->isConnected = 0;
    }

    break;
  }

  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));

  return status;
}

/* -------------------------------------------------------------------------- */
/* LOCAL FUNCTIONS - IMPLEMENTATION                                           */
/* -------------------------------------------------------------------------- */

/**
 * @implements lConvertSocketIp
 *
 **/
#if 0
static TBoolean lConvertSocketIp
(
  const uint8_t*  xpAddress,
  TKSocketIp*     xpSocketIp
)
{
  return E_FALSE;
}

#endif
/* -------------------------------------------------------------------------- */
/* END OF FILE                                                                */
/* -------------------------------------------------------------------------- */



// Uses TCP layer
#else


/*******************************************************************************
*************************keySTREAM Trusted Agent ("KTA")************************

* (c) 2023-2024 Nagravision SÃ rl

* Subject to your compliance with these terms, you may use the Nagravision SÃ rl
* Software and any derivatives exclusively with Nagravision's products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may accompany
* Nagravision Software.

* Redistribution of this Nagravision Software in source or binary form is allowed
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.

* THIS SOFTWARE IS SUPPLIED BY NAGRAVISION "AS IS". NO WARRANTIES, WHETHER EXPRESS,
* IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
* NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN NO
* EVENT WILL NAGRAVISION BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL
* OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF NAGRAVISION HAS BEEN ADVISED OF THE
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW,
* NAGRAVISION 'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
* SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY
* TO NAGRAVISION FOR THIS SOFTWARE.
********************************************************************************/
/** \brief  Interface for socket communication.
 *
 *  \author Kudelski IoT
 *
 *  \date 2023/06/12
 *
 *  \file k_sal_com.c
 ******************************************************************************/
/**
 * @brief Interface for socket communication.
 */

#include "k_sal_com.h"
/* -------------------------------------------------------------------------- */
/* IMPORTS                                                                    */
/* -------------------------------------------------------------------------- */
//#include "cloud_wifi_task.h"
//#include "socket.h"
#include "cryptoauthlib.h"

#include "myprintf.h"

#include <errno.h>
#include <unistd.h>
#include <string.h>

#include "comm_if.h"


/* -------------------------------------------------------------------------- */
/* LOCAL CONSTANTS, TYPES, ENUM                                               */
/* -------------------------------------------------------------------------- */

/** @brief Invalid Socket Id. */
#define C_SAL_COM_SOCKET_INVALID (-1)

/** @brief Connect wait time in ms. */
#define C_SAL_COM_CONNECT_DELAY_IN_MS       (500u)

/**
 * @brief
 * Reset read time out value in ms(this will override the read timeout
 * after first successful read. This is added to avoid more dealy in read because
 * read will always hit the timeout while checking any bytes are available or not.
 */
#define C_SAL_COM_RESET_READ_TIMEOUT_IN_MS  (100u)

/** @brief Length of an address in IP V6 protocol, in bytes. */
#define C_SAL_COM_V6_ADDRESS_NUM_BYTES      (16u)

/******************************************************************************/
/* LOCAL MACROS                                                               */
/******************************************************************************/

#define DEBUG1  //[AB]
#ifdef DEBUG1
/** @brief Enable sal com logs. */
#define M_INTL_SAL_COM_DEBUG(__PRINT__)  do { \
                                              myprintf("\tCOM %d>", __LINE__); \
                                              myprintf __PRINT__; \
                                              myprintf("\r\n"); \
                                          } while (0)
#define M_INTL_SAL_COM_ERROR(__PRINT__)  do { \
                                              myprintf("\tCOM %d> ERROR ", __LINE__); \
                                              myprintf __PRINT__; \
                                              myprintf("\r\n"); \
                                          } while (0)
#else
#define M_INTL_SAL_COM_DEBUG(__PRINT__)
#define M_INTL_SAL_COM_ERROR(__PRINT__)
#endif /* DEBUG. */

/******************************************************************************/
/* TYPES & STRUCTURES                                                         */
/******************************************************************************/

/** @brief Boolean Types. */
typedef enum
{
  E_FALSE,
  /* TRUE. */
  E_TRUE,
  /* FALSE. */
  E_UNDEF
  /* Undefined. */
} TBoolean;

/**
 * @ingroup g_k_types
 *
 * @brief Supported IP protocols.
 */
typedef enum
{
  E_K_IP_PROTOCOL_V4,
  /* IP V4. */
  E_K_IP_PROTOCOL_V6,
  /* IP V6. */
  E_K_IP_NUM_PROTOCOLS
  /* Number of supported IP protocols. */
} TKIpProtocol;

/**
 * @ingroup g_k_types
 *
 * @brief Socket IP address, expressed for protocol IP V4.
*/
typedef struct
{
  uint32_t       address;
  /* IP V4 address, MSBF; e.g. 192.168.15.4 = 0xC0A80F04 */
  uint16_t       port;
  /* IP port. */
} TKSocketAddressV4;

/**
 * @ingroup g_k_types
 *
 * @brief Socket IP.
 */
typedef struct
{
  TKIpProtocol         protocol;
  /* IP protocol. */
  TKSocketAddressV4    v4;
  /* IP address; the used field depends on protocol value. */
} TKSocketIp;

/**
 * @ingroup g_k_types
 *
 * @brief Communication info.
 */
typedef struct
{
  int       socketId;
  /* ID of unique socket used. */
  uint32_t  connectTimeOut;
  /* Connection timeout. */
  uint32_t  readTimeOut;
  /* Read timeout. */
  uint32_t  isConnected;
  /* Flag to check whether already connection estabilished with server or not. */
} TKComInfo;

/* -------------------------------------------------------------------------- */
/* LOCAL VARIABLES                                                            */
/* -------------------------------------------------------------------------- */
static TKComInfo gComInfo = {C_SAL_COM_SOCKET_INVALID, 0, 0, 0};
/* -------------------------------------------------------------------------- */
/* LOCAL FUNCTIONS - PROTOTYPE                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief
 *   Convert the ip to sal socket structure.
 *
 * @param[in] xpAddress
 *   Ip address of the server. Should not be NULL
 * @param[out] xpSocketIp
 *   Converted SAL socket ip.
 * @return
 * - E_TRUE for success
 * - E_FALSE for failure
 */
#if 0
static TBoolean lConvertSocketIp
(
  const uint8_t*  xpAddress,
  TKSocketIp*     xpSocketIp
);

#endif

/* -------------------------------------------------------------------------- */
/* PUBLIC VARIABLES                                                           */
/* -------------------------------------------------------------------------- */

extern APP_DATA appData;

/* -------------------------------------------------------------------------- */
/* PUBLIC FUNCTIONS - IMPLEMENTATION                                          */
/* -------------------------------------------------------------------------- */

TKCommStatus GetIpFromHost(const uint8_t* dnsLookupTargetHost, IP_MULTI_ADDRESS* ipAddr)
{
  TCPIP_DNS_RESULT dnsResult = TCPIP_DNS_RES_PENDING;
  TKCommStatus status = E_K_COMM_STATUS_ERROR;
  uint32_t dnsLookUpStartTick = SYS_TMR_TickCountGet();
  uint32_t timeout = 0;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));

  for (;;)
  {
    if ((NULL == dnsLookupTargetHost) || (NULL == ipAddr))
    {
      M_INTL_SAL_COM_ERROR(("Invalid parameter"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }

    M_INTL_SAL_COM_DEBUG(("Resolving hostname %s", dnsLookupTargetHost));
    TCPIP_DNS_Resolve((const char *)dnsLookupTargetHost, TCPIP_DNS_TYPE_A);
    // TODO: Retries?
    while(TCPIP_DNS_RES_PENDING == dnsResult)
    {
      timeout = (SYS_TMR_TickCountGet() - dnsLookUpStartTick) / SYS_TMR_TickCounterFrequencyGet();
      // TODO: #include configuration.h (for TCPIP_DNS_CLIENT_SERVER_TMO)
      if(timeout >= (TCPIP_DNS_CLIENT_SERVER_TMO / 2))
      {
        M_INTL_SAL_COM_ERROR(("DNS Lookup: request timeout"));
        break;
      }

      atca_delay_ms(5);
      dnsResult = TCPIP_DNS_IsResolved((const char *)dnsLookupTargetHost, ipAddr, TCPIP_DNS_TYPE_A);
    }

    if ((TCPIP_DNS_RES_OK == dnsResult) || (TCPIP_DNS_RES_NAME_IS_IPADDRESS == dnsResult))
    {
       M_INTL_SAL_COM_DEBUG(("Resolved hostname %s as IP %u.%u.%u.%u",
                             (const char *)dnsLookupTargetHost,
                             ipAddr->v4Add.v[0],
                             ipAddr->v4Add.v[1],
                             ipAddr->v4Add.v[2],
                             ipAddr->v4Add.v[3]));
      status = E_K_COMM_STATUS_OK;
    }
    else
    {
      M_INTL_SAL_COM_ERROR(("DNS Lookup failed with status %d", dnsResult));
    }

    break;
  }

  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));

  return status;
}


/**
 * @brief  implement salComInit
 *
 */
K_SAL_API TKCommStatus salComInit
(
  uint32_t  xConnectTimeoutInMs,
  uint32_t  xReadTimeoutInMs,
  void**    xppComInfo
)
{
  TKCommStatus status = E_K_COMM_STATUS_ERROR;
  TKComInfo* pComInfo = NULL;

  M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));
  for (;;)
  {
    if (NULL == xppComInfo)
    {
      M_INTL_SAL_COM_ERROR(("Invalid parameter"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }

    pComInfo = &gComInfo;
    pComInfo->socketId = C_SAL_COM_SOCKET_INVALID;
    pComInfo->connectTimeOut = xConnectTimeoutInMs;
    pComInfo->readTimeOut = xReadTimeoutInMs;
    pComInfo->isConnected = 0;

    *xppComInfo = pComInfo;
    
    status = E_K_COMM_STATUS_OK;
    break;
  }
  M_INTL_SAL_COM_DEBUG(("End of %s", __func__));
  return status;
}

/**
 * @brief  implement salComConnect
 *
 */
K_SAL_API TKCommStatus salComConnect
(
  void*          xpComInfo,
  const uint8_t* xpHost,
  const uint8_t* xpPort
)
{ 
  TKComInfo*   pComInfo = (TKComInfo *)xpComInfo;
  TKCommStatus status = E_K_COMM_STATUS_ERROR;
  IP_MULTI_ADDRESS addr     = { 0 };
  
  for (;;)
  {
    if (E_K_COMM_STATUS_OK != GetIpFromHost(xpHost, &addr))
    {
      M_INTL_SAL_COM_ERROR(("Failed to resolve URL to ip address"));
      status = E_K_COMM_STATUS_PARAMETER;
      break;
    }
      
    pComInfo->socketId = TCPIP_TCP_ClientOpen(IP_ADDRESS_TYPE_IPV4, C_K_COMM__SERVER_PORT, &addr);     
    if (INVALID_SOCKET == pComInfo->socketId)
    {
      M_INTL_SAL_COM_ERROR(("Connect to server failed"));
      status = E_K_COMM_STATUS_ERROR;
      break;
    }
    TCPIP_TCP_WasReset(pComInfo->socketId);
    
    while (!TCPIP_TCP_IsConnected(pComInfo->socketId))
    {
        vTaskDelay(100U / portTICK_PERIOD_MS);
    }
    
    M_INTL_SAL_COM_DEBUG(("Connected to server"));

    status = E_K_COMM_STATUS_OK;
    break;
  }

    return status;
}

/**
 * @brief  implement salComWrite
 *
 */
K_SAL_API TKCommStatus salComWrite
(
  void*          xpComInfo,
  const uint8_t* xpBuffer,
  size_t         xBufferLen
)
{
    TKComInfo*   pComInfo = (TKComInfo *)xpComInfo;
    
    M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));
  
    if(TCPIP_TCP_PutIsReady(pComInfo->socketId) == 0)
    {
        M_INTL_SAL_COM_ERROR(("TCP TX buffer full"));
        return E_K_COMM_STATUS_ERROR;
    }    

    TCPIP_TCP_ArrayPut(pComInfo->socketId, (uint8_t*)xpBuffer, xBufferLen);
    
    M_INTL_SAL_COM_DEBUG(("Sent [%d] bytes", xBufferLen));

    return E_K_COMM_STATUS_OK;   
}

/**
 * @brief  implement salComRead
 *
 */
K_SAL_API TKCommStatus salComRead
(
  void*     xpComInfo,
  uint8_t*  xpBuffer,
  size_t*   xpBufferLen
)
{
    TKComInfo*   pComInfo = (TKComInfo *)xpComInfo;
    
    M_INTL_SAL_COM_DEBUG(("Start of %s", __func__));
  
    if(!TCPIP_TCP_IsConnected(pComInfo->socketId))
    {
        M_INTL_SAL_COM_ERROR(("Not connected to server"));
        return E_K_COMM_STATUS_ERROR;
    }    
    
    M_INTL_SAL_COM_DEBUG(("Fetching [%d] bytes", *xpBufferLen));
    
    uint16_t bytesReceived = 0;
    do
    {    
        bytesReceived = TCPIP_TCP_ArrayGet(pComInfo->socketId, xpBuffer, *xpBufferLen);
        vTaskDelay(10U / portTICK_PERIOD_MS);
    } while (bytesReceived == 0);
    
    *xpBufferLen = bytesReceived;
    
    return E_K_COMM_STATUS_OK;
}

/**
 * @brief  implement salComTerm
 *
 */
K_SAL_API TKCommStatus salComTerm
(
  void*  xpComInfo
)
{
    TKComInfo*   pComInfo = (TKComInfo *)xpComInfo;
    
    TCPIP_TCP_Close(pComInfo->socketId);
    
    return E_K_COMM_STATUS_OK;
}

/* -------------------------------------------------------------------------- */
/* LOCAL FUNCTIONS - IMPLEMENTATION                                           */
/* -------------------------------------------------------------------------- */

/**
 * @implements lConvertSocketIp
 *
 **/
#if 0
static TBoolean lConvertSocketIp
(
  const uint8_t*  xpAddress,
  TKSocketIp*     xpSocketIp
)
{
  return E_FALSE;
}

#endif
/* -------------------------------------------------------------------------- */
/* END OF FILE                                                                */
/* -------------------------------------------------------------------------- */

#endif