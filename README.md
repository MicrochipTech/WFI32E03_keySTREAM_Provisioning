# keySTREAM Provisioning IoT Cloud Demo <!-- omit in toc -->

<img src="images/IoT-Made-Easy-Logo.png" width=100>

> "IoT Made Easy!" - This application demonstrates the in-field provisioning and registration to a Cloud-Computing provider by means of the ECC608-TMNGTLS CryptoAuthentication&trade; element and Kudelski keySTREAM Trusted Agent (KTA).

Devices: **| WFI32E03 | ECC608-TMNGTLS |**<br>
Features: **| Wi-Fi |**


## ⚠ Disclaimer <!-- omit in toc -->

<p><span style="color:red"><b>
THE SOFTWARE ARE PROVIDED "AS IS" AND GIVE A PATH FOR SELF-SUPPORT AND SELF-MAINTENANCE. This repository contains example code intended to help accelerate client product development. </br>

For additional Microchip repos, see: <a href="https://github.com/Microchip-MPLAB-Harmony" target="_blank">https://github.com/Microchip-MPLAB-Harmony</a>

Checkout the <a href="https://microchipsupport.force.com/s/" target="_blank">Technical support portal</a> to access our knowledge base, community forums or submit support ticket requests.
</span></p></b>

## Contents 

- [Contents](#contents)
- [Introduction](#introduction)
- [Bill of Material](#bill-of-material)
- [Prerequisites](#prerequisites)
- [Hardware Setup](#hardware-setup)
    - [PIC32 WFI32 2.0 Curiosity Board](#pic32-wfi32-20-curiosity-board)
    - [ATECC608 TRUST Board](#atecc608-trust-board)
    - [CryptoAuth TrustMANAGER Board](#cryptoauth-trustmanager-board)
    - [MPLAB® PICkit™ 4 In-Circuit Debugger](#mplab-pickit-4-in-circuit-debugger)
- [Software Setup](#software-setup)
- [Firmware](#firmware)
- [Links](#links)


## Introduction

This application demonstrates how to onboard a device to a WiFi network via 




BLE and open a TCP client in Host Companion mode using the WBZ451 Curiosity board. In this application, the WINCS02 module will host a TCP client in Wi-Fi STA mode. Users can either pre-configure the Wi-Fi credentials for their Home-AP or provision them through BLE using the Microchip Bluetooth Data (MBD) app.

To ensure proper remote TCP server configuration, provide the necessary details. By default, the application will connect to the Home-AP and initiate a TCP client socket connection with a TCP server. Once a successful TCP server-client connection is established, data exchange will commence.

Users can utilize the "Microchip Bluetooth Data" mobile application to send the AP credentials to the WINCS02 module via a custom BLE service created on the WBZ451 device.

[TOP](#contents)

## Bill of Material

| TOOLS | QUANTITY |
| :- | :- |
| [PIC32 WFI32 2.0 Curiosity Board](https://www.microchip.com/en-us/development-tool/EV67T15A) | 1 |
| [ATECC608 TRUST Board Revision: 04-11017-R4 and above](https://www.microchip.com/en-us/development-tool/DT100104) | 1 |
| [CryptoAuth TrustMANAGER Board](https://www.microchip.com/en-us/development-tool/EV10E69A) | 1 |
| [MPLAB® PICkit&trade; 4 In-Circuit Debugger](https://www.microchip.com/en-us/development-tool/PG164140) | 1 |

[TOP](#contents)

## Prerequisites

- Create AWS account
- Create Kudelski IoT keySTREAM account
- Before utilizing the target hardware platform the ECC608-TMNGTLS CryptoAuthentication&trade; security element (<i>TrustMANAGER</i>) needs to be registered at Kudelski IoT keySTREAM initially. Follow the instructions as described in setting up the [ATECC608 TRUST](#atecc608-trust-board) and [CryptoAuth TrustMANAGER](#cryptoauth-trustmanager-board) board. <br>Open the Microchip Trust Platform Design Suite (TPDS) and select <i>Usecases</i>.<br><br>
<img src="images/Tpds.png" width=400 style="display: block; margin: auto; float: center"><br> Now select <i>"CryptoAuth Trust Platform - TMNG"</i> in the <i>Kit</i> drop-down box and <i>"keySTREAM&trade; In-field Provisioning"</i> as the <i>Usecase</i>.<br>In the next dialog click the first <i>Pre-Config instruction</i> to generate the Manifest file for the ATECC608 secure element. The second step requires device related information from Kudelski IoT keySTREAM and AWS. Use the <i>Usecase Help</i> button and follow the guidance to create and gather the necessary data.<br>
<img src="images/UsecaseHelp.png" width=500 style="display: block; margin: auto; float: center"><br> Finish the remaining steps 3 to 6. On success, close TPDS and open the MPLAB X project separately. <br> Follow the instructions as described in setting up the [PIC32 WFI32 2.0 Curiosity board](#pic32-wfi32-20-curiosity-board).

[TOP](#contents)

## Hardware Setup
#### <u>PIC32 WFI32 2.0 Curiosity Board</u>

  - Connect the ATECC608 TRUST and PIC32 WFI32 2.0 Curiosity board via mikroBUS&trade; header (J200)
  - For normal operations set the Power Source Selection Jumper (J202) to V<sub>BUS</sub>-VIN (5-6), if the demo software has been already programmed to the device
  - Connect the Target VBUS Micro-B Connector (J204) on the board to the computer using a micro USB cable
  - On the GPIO Header (J207), connect U1RX (PIN 13) and U1TX (PIN 23) to TX and RX pin of any USB to UART converter. When using FTDI chips, connect GND (PIN 17) additionally.
  - Home AP (Wi-Fi Access Point with internet connection)
  - For device programming, follow the instruction as descibed in setting up the [MPLAB® PICkit&trade; 4 In-Circuit Debugger](#mplab-pickit-4-in-circuit-debugger)

#### <u>ATECC608 TRUST Board</u>

  - To activate the <i>TrustMANAGER</i> secure element of board revision #4 set DIP switch 8 to ON (SW2)
  - To activate the <i>TrustMANAGER</i> secure element of board revision #5 or later set DIP switch 5 to ON (SW2)

#### <u>CryptoAuth TrustMANAGER Board</u>

  - Set DIP switch SW2_1 to ON to enable mikroBUS&trade; header and SW2_2 to OFF to disable the on-board devices
  - Connect the ATECC608 and CryptoAuth TrustMANAGER board via mikroBUS&trade; header
  - Connect the board to the computer using a micro USB cable

#### <u>MPLAB® PICkit&trade; 4 In-Circuit Debugger</u>

  - Set the Power Source Selection Jumper (J202) to PKOB-VIN (3-4)
  - Connect the PKOB3 Micro-B USB connector (J302) on the board to the computer using a micro USB cable
  - Connect the debugger to ISCP&trade; header (J206)
  - Connect the debugger to the computer using a micro USB cable

[TOP](#contents)

## Software Setup
<u>Development Tools:</u>
  - MPLAB® X IDE v6.20
  - MPLAB® X IDE plug-ins: MPLAB® Code Configurator (MCC) v5.7.1 and above
  - MPLAB® XC32 C/C++ Compiler v4.10
  - MPLAB® Harmony v3
  - Device Pack: PIC32MZ-W_DFP (1.8.326)
	
<u>MCC Content Library:</u>
|  Harmony v3 Component | version |
| :- | :- |
| bsp | v3.22.0 |
| csp | v3.19.6 |
| core | v3.13.5 |
| paho.mqtt.embedded-c | v1.2.3 |
| keySTREAM_provisioning | v1.0.1 |
| cryptoauthlib | v3.7.5 |
| wolfssl | v5.4.0 |
| wolfMQTT | v1.19.2 |
| net | v3.12.2 |
| crypto | v3.8.2 |
| wireless_wifi | v3.11.1 |
| wireless_system_pic32mzw1_wfi32e01 | v3.9.1 |
| CMSIS_5 | v5.9.0 |
| CMSIS-FreeRTOS | v11.0.1 |

<u>others:</u>
  - Microchip Trust Platform Design Suite (TPDS)
  - Serial Terminal application like TERA TERM

[TOP](#contents)

## Firmware
- The Harmony MCC <i>Project Graph</i>'s below depicts the components utilized in this project.<br>
<img src="images/System_Configuration.png" width=600 style="display: block; margin: auto; float: center">
<img src="images/Root_Configuration.png" width=600 style="display: block; margin: auto; float: center"><br>
- For Wi-Fi connectivity insert the credentials and make sure the router is connected to the internet.<br>
<img src="images/WifiService_Credentials.png" width=450 style="display: block; margin: auto; float: center">
- Configure the Net service.<br>
<img src="images/NetService_Configuration.png" width=450 style="display: block; margin: auto; float: center">
- Configure the ATECC608.<br>
<img src="images/Atecc608_Configuration.png" width=500 style="display: block; margin: auto; float: center">
- Insert the <i>Fleet Profile Public ID</i> as entered during the <i>Create Fleet Profile</i> process.<br>
<img src="images/KtaLib_Configuration.png" width=500 style="display: block; margin: auto; float: center">
- Configure the MQTT service. As SSID enter "aaibu8os7tajk-ats.iot.us-east-2.amazonaws.com"<br>
<img src="images/MqttService_Configuration.png" width=500 style="display: block; margin: auto; float: center">
- Configure the Core - Crypto Authentication library.<br>
<img src="images/CryptoAuthLibrary_Configuration1.png" width=500 style="display: block; margin: auto; float: center">
<img src="images/CryptoAuthLibrary_Configuration2.png" width=500 style="display: block; margin: auto; float: center"><br>
- Configure the wolfCrypt and wolfSSL library.<br>
<img src="images/wolfCryptLibrary_Configuration.png" width=450 style="display: block; margin: auto; float: center">
<img src="images/wolfSslLibrary_Configuration.png" width=320 style="display: block; margin: auto; float: center">
- Configure the on-board LEDs.<br>
<img src="images/Led_Configuration.png" width=700 style="display: block; margin: auto; float: center"><br>
  
[TOP](#contents)

## Links

[TOP](#contents)