
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
#include "BlueNRG1_adc.h"
/* Private includes -----------------------------------------------------------*/
#include "tmp117.h"
#include "bme680.h"
#include "TSL2591.h"
/* Private typedef -----------------------------------------------------------*/
#define ADC_CONVERSION    (ADC_ConversionMode_Single)
#define ADC_ENABLE()      (ADC_Cmd(ENABLE))
#define ADC_CONFIGURATION()   (ADC_Configuration())
#define ADC_CHECK_FLAG        (ADC_GetFlagStatus(ADC_FLAG_EOC))
#define BLE_SENSOR_TYPE 0x02
#define BLE_SENSOR_NUMBER 0x01
#undef TEST_TONE
/* Private define ------------------------------------------------------------*/
#define BLE_BEACON_VERSION_STRING "1.1.0"

/* Set to 1 for enabling Flags AD Type position at the beginning 
   of the advertising packet */
#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitType xADC_InitType;
uint8_t uuid_buffer[30];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#pragma location=".version"

#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
#define vsize  0
__root const uint8_t version[15] = {0x02, 0x01, 0x06, 26, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                                    0x4C, 0x00, 0x02, 0x15,
                                    0x23, 0x32, 0xA4, 0xC2,
                                    BLE_SENSOR_TYPE, BLE_SENSOR_NUMBER};
#else
#define vsize  3
__root const uint8_t version[12] = {26, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                                    0x4C, 0x00, 0x02, 0x15,
                                    0x23, 0x32, 0xA4, 0xC2,
                                    BLE_SENSOR_TYPE, BLE_SENSOR_NUMBER};
#endif
void get_sensor_data(uint8_t);
void ADC_Configuration(void);
void ADC_NVIC_Configuration(void);


void Device_Init(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  
  /* Set the TX Power to 8 dBm */
  ret = aci_hal_set_tx_power_level(1,6);
  if(ret != 0) {
    while(1);
  }
#ifdef TEST_TONE
  aci_hal_tone_start(0,0);
  while(1);
#endif
  
  /* Init the GATT */
  ret = aci_gatt_init();
  /* Init the GAP */
  ret = aci_gap_init(0x01, 0x00, 0x08, &service_handle, 
                     &dev_name_char_handle, &appearance_char_handle);
}

/**
* @brief  Start beaconing
* @param  None 
* @retval None
*/
static void Start_Beaconing(void)
{  
  uint8_t ret = BLE_STATUS_SUCCESS;
   
  /* disable scan response */
  ret = hci_le_set_scan_response_data(0,NULL);
  if (ret != BLE_STATUS_SUCCESS)  return;
  
  /* put device in non connectable mode */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND, 160, 160, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                 0, NULL,0, NULL, 0, 0); 
  if (ret != BLE_STATUS_SUCCESS)  return;

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

  /* Application demo Led Init */
  SdkEvalLedInit(LED1); //Activity led
  SdkEvalLedOff(LED1); 
  SdkEvalLedInit(LED2); //Activity led
  SdkEvalLedOff(LED2); 
  SdkEvalLedInit(DONE); //DONE PIN 
  SdkEvalLedOff(DONE);
  

  SdkEvalPushButtonInit(ILS_SENSOR); 
  SdkEvalPushButtonInit(ALERT_TMP117); 
  SdkEvalPushButtonInit(INT_TSL25911); 
  SdkEvalI2CInit(100000);
  
  /* BlueNRG-1 stack init */
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) while(1);
  
  /* Init the BlueNRG-1 device */
  Device_Init();
  
  /* Start Beacon Non Connectable Mode*/
  Start_Beaconing();
   
  hci_le_set_advertise_enable(0x00);   //stop advertising 
  
  
    /* BlueNRG-1 stack tick */
  while(1)
  {   
    SdkEvalLedOn(LED2);
    get_sensor_data(SdkEvalPushButtonGetState(ILS_SENSOR)); 
    SdkEvalLedOff(LED2);
    hci_le_set_advertising_data (sizeof(uuid_buffer), uuid_buffer);
    BTLE_StackTick();
    SdkEvalLedOn(LED1); 
    hci_le_set_advertise_enable(0x01);
    BlueNRG_Sleep(SLEEPMODE_RUNNING, 0, 0);
    Clock_Wait(1000);   
    SdkEvalLedOff(LED1); 
    hci_le_set_advertise_enable(0x00); 
    SdkEvalLedOn(DONE);
    Clock_Wait(1000); 
    SdkEvalLedOff(DONE);
  }
    
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


/***************************************************************************************/
/*                         GET DATA FROM SENSOR                                        */
/***************************************************************************************/

void get_sensor_data(uint8_t ils_state)
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
  uint16_t adc_value; 

/******************* GET BATTERY VOLTAGE *********************/   
  /* ADC Initialization */
  ADC_CONFIGURATION();
  /* Start new conversion */
  ADC_ENABLE();
  while(!ADC_CHECK_FLAG);
  /* Read converted data */
  adc_value = (uint16_t)(2000*ADC_GetConvertedData(xADC_InitType.ADC_Input, xADC_InitType.ADC_ReferenceVoltage)); 
  
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
  Clock_Wait(150);
  bme680_get_profile_dur(&meas_period, &gas_sensor);
  rslt = bme680_get_sensor_data(&data, &gas_sensor);
  rslt = bme680_set_sensor_mode(&gas_sensor);
   
  }
 
  /******************* GET DATA FROM TMP117 *********************/

  TMP117_SetConfig(C15mS5,AVE8,DATA,ONESHOT,ACTIVE_H);
  Clock_Wait(1);
  while(SdkEvalPushButtonGetState(ALERT_TMP117) == RESET);
  temperature = (uint16_t)(TMP117_getTemperature()*10+100);
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
  
#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
  uuid_buffer[0] = version[0];
  uuid_buffer[1] = version[1];
  uuid_buffer[2] = version[2];
#endif  
  uuid_buffer[3-vsize] = version[3];
  uuid_buffer[4-vsize] = version[4];
  uuid_buffer[5-vsize] = version[5];
  uuid_buffer[6-vsize] = version[6];  
  uuid_buffer[7-vsize] = version[7];
  uuid_buffer[8-vsize] = version[8]; 
  // PAYLOAD  PREFIX (24bits) =  0x2332A4C2
  uuid_buffer[9-vsize] = version[9];
  uuid_buffer[10-vsize] = version[10];
  uuid_buffer[11-vsize] = version[11];
  uuid_buffer[12-vsize] = version[12];  
   // PAYLOAD  SENSOR TYPE
  uuid_buffer[13-vsize] = version[13];
  // PAYLOAD  SENSOR NUMBER
  uuid_buffer[14-vsize] = version[14]; 
  // PAYLOAD  BATTERY VOLTAGE  :adc_value (5182 = 5.182V)
  uuid_buffer[15-vsize] = (uint8_t)(adc_value>>8 & 0xFF);
  uuid_buffer[16-vsize] = (uint8_t)(adc_value & 0xFF);
 
  // PAYLOAD  TEMPERATURE BME680 (374 = 27.4°C)
  tmp = (uint16_t)(data.temperature+1000.0)/10;
  uuid_buffer[17-vsize] = (uint8_t)(tmp>>8 & 0xFF);
  uuid_buffer[18-vsize] = (uint8_t)(tmp & 0xFF);
    
  // PAYLOAD LIMINOSITY TSL2591(1536 = 1536 RELATIVE ILLUMINANCE)
  tmp = (uint16_t)(lum);
  uuid_buffer[19-vsize] = (uint8_t)(tmp>>8 & 0xFF);    
  uuid_buffer[20-vsize] = (uint8_t)(tmp & 0xFF);       
  
  // PAYLOAD PRESSURE BME680 (9809 = 980.9Hpa)
  tmp = (uint16_t)(data.pressure/10);
  uuid_buffer[21-vsize] = (uint8_t)(tmp>>8 & 0xFF);
  uuid_buffer[22-vsize] = (uint8_t)(tmp & 0xFF);  
  // PAYLOAD GAZ RESISTANCE BME680 (645 = 645KOhms)
  tmp = (uint16_t)(data.gas_resistance/1000);
  uuid_buffer[23-vsize] = (uint8_t)(tmp>>8 & 0xFF);
  uuid_buffer[24-vsize] = (uint8_t)(tmp & 0xFF); 

  // PAYLOAD  TEMPERATURE TMP117 (375 = 27.5°C ;;; 1375 = 27.5°C + ILS_Sensor)
  if (!ils_state) temperature+=1000;
  uuid_buffer[25-vsize] = (uint8_t)(temperature>>8 & 0xFF);  // MAJOR
  uuid_buffer[26-vsize] = (uint8_t)(temperature & 0xFF);     // MAJOR  

  // PAYLOAD HUMIDITY BME680  (3850 = 38.5% Hum)
  tmp = (uint16_t)(data.humidity/10);                   // MINOR
  uuid_buffer[27-vsize] = (uint8_t)(tmp>>8 & 0xFF);
  uuid_buffer[28-vsize] = (uint8_t)(tmp & 0xFF);              // MINOR

  //2's complement of the Tx power (-56dB)};
  uuid_buffer[29-vsize] = 0xC8;        //2's complement of the Tx power (-56dBm)}; 
  asm("nop");   

  
}

/*********************************************************************************/
/*                                       ADC                                     */
/*********************************************************************************/
void ADC_Configuration(void)
{ 
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);
  
  /* Configure ADC */
  xADC_InitType.ADC_OSR = ADC_OSR_200;
  xADC_InitType.ADC_Input = ADC_Input_AdcPin1;
  xADC_InitType.ADC_ConversionMode = ADC_CONVERSION;
  xADC_InitType.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6;
  xADC_InitType.ADC_Attenuation = ADC_Attenuation_9dB54;
  
  ADC_Init(&xADC_InitType);
  
  /* Enable auto offset correction */
  ADC_AutoOffsetUpdate(ENABLE);
  ADC_Calibration(ENABLE);
}

void ADC_NVIC_Configuration(void)
{
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MED_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
