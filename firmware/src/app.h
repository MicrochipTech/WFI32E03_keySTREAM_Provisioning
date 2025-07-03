/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
//Tera Term Color coding 
#define TERM_BLACK "\x1B[30m"
#define TERM_RED   "\x1B[31m"
#define TERM_GREEN "\x1B[32m"
#define TERM_YELLOW "\x1B[33m"
#define TERM_BLUE "\x1B[34m"
#define TERM_PURPLE "\x1B[35m"
#define TERM_CYAN "\x1B[36m"
#define TERM_WHITE "\x1B[47m"
#define TERM_RESET "\x1B[0m"
#define TERM_BG_RED "\x1B[41m" 
#define TERM_BOLD "\x1B[1m" 
#define TERM_UL "\x1B[4m"

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* Application's state machine's initial state. */
    APP_STATE_INIT = 0,

    // State to perform regular service tasks and wait for callback
    APP_STATE_SERVICE_TASKS,
            
    // Initial state of the application
    APP_STATE_INIT_ECC608,

    // State to open the Wi-Fi driver
    APP_STATE_OPEN_WIFI_DRIVER,

    // State to retrieve device information
    APP_STATE_DEVICE_INFO,

    // State to set the regulatory domain for Wi-Fi
    APP_STATE_SET_WIFI_REG_DOMAIN,

    // State to configure Wi-Fi parameters
    APP_STATE_SET_WIFI_PARAMS,
            
    // State to disconnect Wi-Fi connection
    APP_STATE_WIFI_DISCONNECT,
    
    // State to create socket
    APP_STATE_CREATE_SOCKET,

    // State to handle errors
    APP_STATE_ERROR,

    // State to initialize the secure module
    APP_STATE_INIT_CRYPTOAUTHLIB,
    
    // State to connect to keystream trust agent
    APP_STATE_CONNECT_TO_KEYSTREAM,
            
    /* Application states for cloud activities */
    // Initial cloud state
    APP_STATE_CLOUD_INIT,

    // State indicates connection to cloud establish
    APP_STATE_CLOUD_CONNECTED,
            
    // State indicate subscribing to specific cloud topic
    APP_STATE_CLOUD_SUBSCRIBING,
            
    // State indicates subscription to specific cloud topic
    APP_STATE_CLOUD_SUBSCRIBED
            
} APP_STATES;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES state;

    /* TODO: Define any additional data used by the application. */

} APP_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );

void APP_DebugPrintf(const char* format, ...);

char APP_HexToChar(uint8_t hex);

void APP_DebugPrintBuffer(const uint8_t *pBuf, uint16_t bufLen);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_H */

/*******************************************************************************
 End of File
 */

