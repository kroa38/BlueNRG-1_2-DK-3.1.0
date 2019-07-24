
/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : BLE_Beacon_main.c
* Author             : RF Application Team
* Version            : 1.1.0
* Date               : 15-January-2016
* Description        : Code demostrating the BLE Beacon application
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_Beacon_main.c
 * @brief This is a BLE beacon demo that shows how to configure a BlueNRG-1,2 device 
 * in order to advertise specific manufacturing data and allow another BLE device to
 * know if it is in the range of the BlueNRG-1 beacon device. 
 * It also provides a reference example about how using the 
 * BLE Over-The-Air (OTA) Service Manager firmware upgrade capability.
 * 

* \section ATOLLIC_project ATOLLIC project
  To use the project with ATOLLIC TrueSTUDIO for ARM, please follow the instructions below:
  -# Open the ATOLLIC TrueSTUDIO for ARM and select File->Import... Project menu. 
  -# Select Existing Projects into Workspace. 
  -# Select the ATOLLIC project
  -# Select desired configuration to build from Project->Manage Configurations
  -# Select Project->Rebuild Project. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt> ...\\Project\\BLE_Examples\\BLE_Beacon\\MDK-ARM\\BlueNRG-1\\BLE_Beacon.uvprojx </tt> or
     <tt> ...\\Project\\BLE_Examples\\BLE_Beacon\\MDK-ARM\\BlueNRG-2\\BLE_Beacon.uvprojx </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt> ...\\Project\\BLE_Examples\\BLE_Beacon\\EWARM\\BlueNRG-1\\BLE_Beacon.eww </tt> or
     <tt> ...\\Project\\BLE_Examples\\BLE_Beacon\\EWARM\\BlueNRG-2\\BLE_Beacon.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration
- \c Use_OTA_ServiceManager - Configuration for Application using OTA Service Manager

     
* \section Board_supported Boards supported
- \c STEVAL-IDB007V1
- \c STEVAL-IDB007V2
- \c STEVAL-IDB008V1
- \c STEVAL-IDB008V2
- \c STEVAL-IDB009V1


 * \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB00XV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name |            |  Description                                                                |
| JP1         |   JP2      |                                                                             |
----------------------------------------------------------------------------------------------------------
| ON 1-2      | ON 2-3     | USB supply power                                                            |
| ON 2-3      | ON 1-2     | The supply voltage must be provided through battery pins.                   |
| ON 1-2      |            | USB supply power to STM32L1, JP2 pin 2 external power to BlueNRG1           |


@endtable 

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB00XV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | 1-2: to provide power from USB (JP2:2-3). 2-3: to provide power from battery holder (JP2:1-2)                                                                          |          
| JP2         | 1-2: to provide power from battery holder (JP1:2-3). 2-3: to provide power from USB (JP1:1-2). Pin2 to VDD  to provide external power supply to BlueNRG-1 (JP1: 1-2)   |
| JP3         | pin 1 and 2 UART RX and TX of MCU. pin 3 GND.                                                                                                                          |          
| JP4         | Fitted: to provide VBLUE to BlueNRG1. It can be used also for current measurement.                                                                                     |
| JP5         | Fitted : TEST pin to VBLUE. Not fitted:  TEST pin to GND                                                                                                               |


@endtable 

* \section Pin_settings Pin settings
@table
|            |                                                 Release                                                 |||||                                                        Use_OTA_ServiceManager                                                         |||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |      STEVAL-IDB007V1     |      STEVAL-IDB007V2     |      STEVAL-IDB008V1     |      STEVAL-IDB008V2     |      STEVAL-IDB009V1     |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|    ADC1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    ADC2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO0    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO12    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO13    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO15    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO16    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO17    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO18    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO19    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|     IO2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO20    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO21    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO22    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO23    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO24    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO25    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|     IO3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO5    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO6    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO7    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO8    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    TEST1   |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |

@endtable 

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|            |                                                 Release                                                 |||||                                                                                                     Use_OTA_ServiceManager                                                                                                      |||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |               STEVAL-IDB007V1              |               STEVAL-IDB007V2              |               STEVAL-IDB008V1              |               STEVAL-IDB008V2              |               STEVAL-IDB009V1              |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |

@endtable


* \section Buttons_description Buttons description
@table
|                |                                                 Release                                                 |||||                                                                                                                                                                           Use_OTA_ServiceManager                                                                                                                                                                            |||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |                             STEVAL-IDB007V1                            |                             STEVAL-IDB007V2                            |                             STEVAL-IDB008V1                            |                             STEVAL-IDB008V2                            |                             STEVAL-IDB009V1                            |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                                Not Used                                |                                Not Used                                |                                Not Used                                |                                Not Used                                |                                Not Used                                |
|      RESET     |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |                             Reset BlueNRG1                             |                             Reset BlueNRG1                             |                             Reset BlueNRG2                             |                             Reset BlueNRG2                             |                             Reset BlueNRG2                             |

@endtable

* \section Usage Usage

The Beacon demo configures a BlueNRG-1,2 device in advertising mode (non-connectable mode) with specific manufacturing data.
It transmits advertisement packets at regular intervals which contain the following manufacturing data:
@table   
------------------------------------------------------------------------------------------------------------------------
| Data field              | Description                       | Notes                                                  |
------------------------------------------------------------------------------------------------------------------------
| Company identifier code | SIG company identifier (1)        | Default is 0x0030 (STMicroelectronics)                 |
| ID                      | Beacon ID                         | Fixed value                                            |
| Length                  | Length of the remaining payload   | NA                                                     |
| Location UUID           | Beacons UUID                      | It is used to distinguish specific beacons from others |
| Major number            | Identifier for a group of beacons | It is used to group a related set of beacons           |                                              
| Minor number            | Identifier for a single beacon    | It is used to identify a single beacon                 |                                       
| Tx Power                | 2's complement of the Tx power    | It is used to establish how far you are from device    |                                       
@endtable

 - (1): SIG company identifiers are available on https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
 - NA : Not Applicable;
NOTEs:
     - OTA Service Manager support requires to build application by enabling only ST_USE_OTA_SERVICE_MANAGER_APPLICATION=1 (preprocessor, linker) options and through files: OTA_btl.[ch] (refer to Release_with_OTA_ServiceManager IAR workspace).
     - OTA FW upgrade feature is supported only on BlueNRG-2, BLE stack v2.x.

**/
   
/** @addtogroup BlueNRG1_demonstrations_applications
 *  BlueNRG-1 Beacon demo \see BLE_Beacon_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "sleep.h"
#include "SDK_EVAL_Config.h"
#include "Beacon_config.h"
#include "OTA_btl.h"
#include "clock.h"
#include "tmp117.h"
#include "bme680.h"
#include "TSL2591.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_BEACON_VERSION_STRING "1.1.0"

/* Set to 1 for enabling Flags AD Type position at the beginning 
   of the advertising packet */
#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t adv_data[];
uint8_t uuid_buffer[30];
//volatile uint32_t lSystickCounter=0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void get_sensor_data(void);


void Device_Init(void)
{


  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t device_name[] = {'B', 'l', 'u', 'e', 'N', 'R', 'G'};  
  
  /* Set the TX Power to 8 dBm */
  ret = aci_hal_set_tx_power_level(1,7);
  if(ret != 0) {
    printf ("Error in aci_hal_set_tx_power_level() 0x%04xr\n", ret);
    while(1);
  }

  /* Init the GATT */
  ret = aci_gatt_init();
  if (ret != 0) 
    printf ("Error in aci_gatt_init() 0x%04xr\n", ret);
  else
    printf ("aci_gatt_init() --> SUCCESS\r\n");
  
  /* Init the GAP */
  ret = aci_gap_init(0x01, 0x00, 0x08, &service_handle, 
                     &dev_name_char_handle, &appearance_char_handle);
  if (ret != 0)
    printf ("Error in aci_gap_init() 0x%04x\r\n", ret);
  else
    printf ("aci_gap_init() --> SUCCESS\r\n");

  ret = aci_gatt_update_char_value_ext(0, service_handle, dev_name_char_handle, 0,sizeof(device_name), 0, sizeof(device_name), device_name);
  if(ret != BLE_STATUS_SUCCESS) 
    printf("aci_gatt_update_char_value_ext() failed: 0x%02x\r\n", ret);
  else
    printf ("aci_gatt_update_char_value_ext() --> SUCCESS\r\n");
 
  ret = aci_hal_set_tx_power_level(0x01,0x07);
  if(ret != BLE_STATUS_SUCCESS) 
    printf("aci_hal_set_tx_power_level() failed: 0x%02x\r\n", ret);
  else
    printf ("aci_hal_set_tx_power_level() --> SUCCESS\r\n");
  

}


/**
* @brief  Start beaconing
* @param  None 
* @retval None
*/
static void Start_Beaconing(void)
{  
  uint8_t ret = BLE_STATUS_SUCCESS;

#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
  /* Set AD Type Flags at beginning on Advertising packet  */
  uint8_t adv_data[] = {
      /* Advertising data: Flags AD Type */
      0x02, 
      0x01, 
      0x06, 
      /* Advertising data: manufacturer specific data */
      26, //len
      AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
      0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
      0x02,       // ID
      0x15,       //Length of the remaining payload
      0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
      0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
      0x30, 0x00, // Major number 
      0x10, 0x00, // Minor number 
      0xC8        //2's complement of the Tx power (-56dB)};      
   };
#else
   uint8_t manuf_data[] = {
      26, //len
      AD_TYPE_MANUFACTURER_SPECIFIC_DATA, //manufacturer type
      0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
      0x02,       // ID
      0x15,       //Length of the remaining payload
      0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
      0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
      0x40, 0x00, // Major number 
      0x20, 0x00, // Minor number 
      0xC8        //2's complement of the Tx power (-56dB)};      
   };
#endif
   
  /* disable scan response */
  ret = hci_le_set_scan_response_data(0,NULL);
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in hci_le_set_scan_resp_data() 0x%04x\r\n", ret);
    return;
  }
  else
    printf ("hci_le_set_scan_resp_data() --> SUCCESS\r\n");

  /* put device in non connectable mode */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND, 160, 160, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                 7, "BLUENRG",0, NULL, 0, 0); 
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in aci_gap_set_discoverable() 0x%04x\r\n", ret);
    return;
  }
  else
    printf ("aci_gap_set_discoverable() --> SUCCESS\r\n");

#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
  /* Set the  ADV data with the Flags AD Type at beginning of the 
     advertsing packet,  followed by the beacon manufacturer specific data */
  ret = hci_le_set_advertising_data (sizeof(adv_data), adv_data);
  //ret = hci_le_set_advertising_data (sizeof(uuid_buffer), uuid_buffer);
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in hci_le_set_advertising_data() 0x%04x\r\n", ret);
    return;
  }
  else
    printf ("hci_le_set_advertising_data() --> SUCCESS\r\n");
#else
  /* Delete the TX power level information */
  ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL); 
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in aci_gap_delete_ad_type() 0x%04x\r\n", ret);
    return;
  }
  else
    printf ("aci_gap_delete_ad_type() --> SUCCESS\r\n");

  /* Update the ADV data with the BEACON manufacturing data */
  ret = aci_gap_update_adv_data(27, manuf_data);  
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in aci_gap_update_adv_data() 0x%04x\r\n", ret);
    return;
  }
  else
    printf ("aci_gap_update_adv_data() --> SUCCESS\r\n");
#endif
}

int main(void) 
{
  uint8_t ret;
 
  /* System Init */
  SystemInit();
  /* init clock for delay function */
  Clock_Init();
  /* Identify BlueNRG-1 platform */
  SdkEvalIdentification();

  /* Init the UART peripheral */
  SdkEvalComUartInit(UART_BAUDRATE); 

  /* Application demo Led Init */
  SdkEvalLedInit(LED1); //Activity led
  SdkEvalLedOff(LED1); 

  SdkEvalPushButtonInit(ILS_SENSOR); 
  SdkEvalPushButtonInit(ALERT_TMP117); 
  SdkEvalPushButtonInit(INT_TSL25911); 
  SdkEvalI2CInit(100000);
  //get_sensor_data();                  // get data from BME680, TMP117, TSL2591 etc ...
  
  /* BlueNRG-1 stack init */
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }
  
  /* Init the BlueNRG-1 device */
  Device_Init();
  
#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
  /* Initialize the button */
  SdkEvalPushButtonInit(USER_BUTTON); 
#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
  
  /* Start Beacon Non Connectable Mode*/
  Start_Beaconing();
  
  printf("BlueNRG-1 BLE Beacon Application (version: %s)\r\n", BLE_BEACON_VERSION_STRING); 
  
  printf("Start Advertising\r\n"); 
  hci_le_set_advertise_enable(0x00);   //stop advertising 
  
//  while(1) 
//  {

    /* BlueNRG-1 stack tick */
    BTLE_StackTick();
    printf("Start Advertising\r\n"); 
    hci_le_set_advertise_enable(0x01);
    SdkEvalLedOn(LED1);     
    /* Enable Power Save according the Advertising Interval */
    //BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
    BlueNRG_Sleep(SLEEPMODE_RUNNING, 0, 0);
//    Clock_Wait(1000);   
//    SdkEvalLedOff(LED1); 
//    printf("Stop Advertising\r\n");
//    hci_le_set_advertise_enable(0x00); 
//    Clock_Wait(2000); 
    
     while(1) ;
     
#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
    if (SdkEvalPushButtonGetState(USER_BUTTON) == RESET)
    {
      OTA_Jump_To_Service_Manager_Application();
    }
#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
//  }
}

/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode)
{
  if(SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy())
    return SLEEPMODE_RUNNING;
  
  return SLEEPMODE_NOTIMER;
}

/***************************************************************************************/

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
/**
* @brief  Delay function in ms.
* @param  lTimeMs time in ms
* @retval None
*/

void get_sensor_data(void)
{

  uint16_t meas_period;
  sensor_t sensor;
  uint32_t lum ;
  uint16_t ir, full;
  float lux = 0.;
  int8_t rslt = BME680_OK; 
  uint16_t temperature,tmp;
  uint16_t set_required_settings;
  struct bme680_field_data data;
  struct bme680_dev gas_sensor;
  
   
/******************* GET DATA FROM BME680 *********************/    
    
  gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
  gas_sensor.intf = BME680_I2C_INTF;
  gas_sensor.read = &i2c_read;
  gas_sensor.write = &i2c_write;
  gas_sensor.delay_ms = &user_delay_ms;
  gas_sensor.amb_temp = 25;

  rslt = bme680_init(&gas_sensor);
  
  if(rslt == 0){
    /* Set the temperature, pressure and humidity settings */
  gas_sensor.tph_sett.os_hum = BME680_OS_2X;
  gas_sensor.tph_sett.os_pres = BME680_OS_4X;
  gas_sensor.tph_sett.os_temp = BME680_OS_8X;
  gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

  /* Set the remaining gas sensor settings and link the heating profile */
  gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
  /* Create a ramp heat waveform in 3 steps */
  gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
  gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  gas_sensor.power_mode = BME680_FORCED_MODE; 
  

  /* Set the required sensor settings needed */
  set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

  /* Set the desired sensor configuration */
  rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);

  /* Set the power mode */
  rslt = bme680_set_sensor_mode(&gas_sensor);
  gas_sensor.tph_sett.os_hum = BME680_OS_2X;
  Clock_Wait(300);
  bme680_get_profile_dur(&meas_period, &gas_sensor);
  rslt = bme680_get_sensor_data(&data, &gas_sensor);
  rslt = bme680_set_sensor_mode(&gas_sensor);
   
  }
 
  /******************* GET DATA FROM TMP117 *********************/

  TMP117_SetConfig(C15mS5,AVE8,DATA,ONESHOT,ACTIVE_H);
  Clock_Wait(1);
  while(SdkEvalPushButtonGetState(ALERT_TMP117) == RESET);
  temperature = (uint16_t)(round(TMP117_getTemperature()*100));
  asm("nop"); 
  /******************* GET DATA FROM TSL2591 *********************/  
  
  TSL2591_begin();
  TSL2591_getSensor(&sensor);
  TSL2591_setGain(TSL2591_GAIN_LOW);      // 25x gain
  TSL2591_setTiming(TSL2591_INTEGRATIONTIME_100MS);
  tsl2591Gain_t gain = TSL2591_getGain();
  lum =  TSL2591_getLuminosity(TSL2591_VISIBLE);
  //lum = TSL2591_getFullLuminosity();
  //ir = lum >> 16;
  //full = lum & 0xFFFF;
  //lux = TSL2591_calculateLux(full,ir);
  
  /******************* FILL UUID FRAME **************************/ 

  uuid_buffer[0] = 0x02;
  uuid_buffer[1] = 0x01;
  uuid_buffer[2] = 0x06;
  
  uuid_buffer[3] = 26;
  uuid_buffer[4] = AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
  uuid_buffer[5] = 0x4C;
  uuid_buffer[6] = 0x00;  
  uuid_buffer[7] = 0x02;
  uuid_buffer[8] = 0x15; 
  // PAYLOAD  PREFIX (24bits)
  uuid_buffer[9] = 0x23;
  uuid_buffer[10] = 0x32;
  uuid_buffer[11] = 0xA4;
  uuid_buffer[12] = 0xC2;  
   // PAYLOAD  SENSOR TYPE
  uuid_buffer[13] = 0x02;
  // PAYLOAD  SENSOR NUMBER
  uuid_buffer[14] = 0x01; 
  // PAYLOAD  BATTERY VOLTAGE
  uuid_buffer[15] = 0x00;
  uuid_buffer[16] = 0x00; 
 
  // PAYLOAD  TEMPERATURE BME680
    if(data.temperature<0) 
  {
    tmp = ((uint16_t)data.temperature)/10+2000;
    uuid_buffer[17] = (uint8_t)(tmp>>8 & 0xFF);
    uuid_buffer[18] = (uint8_t)(tmp & 0xFF);
  }
  else
  {     
    tmp = ((uint16_t)data.temperature)/10+1000;
    uuid_buffer[17] = (uint8_t)(tmp>>8 & 0xFF);
    uuid_buffer[18] = (uint8_t)(tmp & 0xFF);
  }  
  // PAYLOAD HUMIDITY BME680
  tmp = (uint16_t)(data.humidity/10);
  uuid_buffer[19] = (uint8_t)(tmp>>8 & 0xFF);
  uuid_buffer[20] = (uint8_t)(tmp & 0xFF);   
  // PAYLOAD PRESSURE BME680
  tmp = (uint16_t)(data.pressure/10);
  uuid_buffer[21] = (uint8_t)(tmp>>8 & 0xFF);
  uuid_buffer[22] = (uint8_t)(tmp & 0xFF);  
  // PAYLOAD GAZ RESISTANCE BME680
  tmp = (uint16_t)(data.gas_resistance/1000);
  uuid_buffer[23] = (uint8_t)(tmp>>8 & 0xFF);
  uuid_buffer[24] = (uint8_t)(tmp & 0xFF); 
  // PAYLOAD LIMINOSITY TSL2591
  tmp = (uint16_t)(lum);
  uuid_buffer[25] = (uint8_t)(tmp>>8 & 0xFF);
  uuid_buffer[26] = (uint8_t)(tmp & 0xFF);   

  uuid_buffer[27] = 0x00;  
  uuid_buffer[28] = 0x00; 
  //2's complement of the Tx power (-56dB)};
  uuid_buffer[29] = 0xC8;        //2's complement of the Tx power (-56dB)}; 
  asm("nop");   

  
}
