#include "TMP117.h"


/*!
 * @file    TMP117.cpp
 * @author  Nils Minor
 * 
 * @license  GNU GENERAL PUBLIC LICENSE (see license.txt)
 * 
 * v1.0.0   - Initial library version
 * 
 * 
 */

#include "TMP117.h"

TMP117_ALERT alert_type;

/*!
    @brief   Initialize in default mode 
    @param   _newDataCallback   callback function will be called when new data is available
*/
  
void TMP117_SetConfig(TMP117_CONVT convtime, TMP117_AVE ave, TMP117_PMODE mode, TMP117_CMODE cmode, TMP117_POL pol)
  {
  uint16_t reg_value = TMP117_readConfig ();
  
  reg_value &= ~((1UL << 9) | (1UL << 8) | (1UL << 7));       // clear bits
  reg_value = reg_value | ( convtime  & 0x07 ) << 7;          // set bits

  reg_value &= ~((1UL << 6) | (1UL << 5) );       // clear bits
  reg_value = reg_value | ( ave & 0x03 ) << 5;    // set bits
 
  if (mode == THERMAL) {
    reg_value |= 1UL << 4;    // change to thermal mode
    reg_value &= ~(1UL << 2); // set pin as alert flag
    reg_value &= ~(1UL << 3); // alert pin low active
  }
  else if (mode == ALERT) {
    reg_value &= ~(1UL << 4); // change to alert mode
    reg_value &= ~(1UL << 2); // set pin as alert flag
    reg_value &= ~(1UL << 3); // alert pin low active
  } 
  else if (mode == DATA) {
    reg_value |= 1UL << 2;    // set pin as data ready flag
  }  
  
  reg_value &= ~(1UL << 3);     // clear bits
  if(pol == ACTIVE_H)
  {
    reg_value |= 1UL << 3;
  }

  reg_value &= ~((1UL << 11) | (1UL << 10));       // clear bits
  reg_value = reg_value | ( cmode  & 0x03 ) << 10; // set bits   
  
  TMP117_writeConfig ( reg_value );
  
  
}

/*!
    @brief    Read configuration register and handle events.
              Should be called in loop in order to call callback functions 
*/
void TMP117_update (void) {
  TMP117_readConfig ();
}

/*!
    @brief   Performs a soft reset. All default values will be loaded to the configuration register
*/
void TMP117_softReset ( void ) {
  uint16_t reg_value = 0;
  reg_value |= 1UL << 1;
  TMP117_writeConfig ( reg_value );
}

/*!
    @brief   Set alert pin mode 
    @param   mode TMP117_PMODE [Thermal-Alert-Data]
*/
void TMP117_setAlertMode ( TMP117_PMODE mode) {
  uint16_t reg_value = TMP117_readConfig ();
  
  if (mode == THERMAL) {
    reg_value |= 1UL << 4;    // change to thermal mode
    reg_value &= ~(1UL << 2); // set pin as alert flag
    reg_value &= ~(1UL << 3); // alert pin low active
  }
  else if (mode == ALERT) {
    reg_value &= ~(1UL << 4); // change to alert mode
    reg_value &= ~(1UL << 2); // set pin as alert flag
    reg_value &= ~(1UL << 3); // alert pin low active
  } 
  else if (mode == DATA) {
    reg_value |= 1UL << 2;    // set pin as data ready flag
  } 
  TMP117_writeConfig ( reg_value );
}


/*!
    @brief    Set alert temperature
    
    @param    lowtemp   low boundary alert temperature
    @param    hightemp  high boundary alert temperature  
*/
void TMP117_setAllertTemperature (double lowtemp, double hightemp) {
  
 uint16_t high_temp_value = (uint16_t)(hightemp / TMP117_RESOLUTION);
 uint16_t low_temp_value = (uint16_t)(lowtemp / TMP117_RESOLUTION);

 TMP117_i2cWrite2B (TMP117_REG_TEMP_HIGH_LIMIT , high_temp_value);
 TMP117_i2cWrite2B (TMP117_REG_TEMP_LOW_LIMIT , low_temp_value);  
}

/*!
    @brief    Set conversion mode
    
    @param    cmode   ::TMP117_CMODE [CONTINUOUS-SHUTDOWN-ONESHOT]
*/
void TMP117_setConvMode ( TMP117_CMODE cmode) {
   uint16_t reg_value = TMP117_readConfig ();
   reg_value &= ~((1UL << 11) | (1UL << 10));       // clear bits
   reg_value = reg_value | ( cmode  & 0x03 ) << 10; // set bits   
   TMP117_writeConfig ( reg_value );
}

/*!
    @brief    Set conversion time
    
    @param    convtime  ::TMP117_CONVT [C15mS5-C125mS-C250mS-C500mS-C1S-C4S-C8S-C16S]
*/
void TMP117_setConvTime ( TMP117_CONVT convtime ) {
  uint16_t reg_value = TMP117_readConfig ();
  reg_value &= ~((1UL << 9) | (1UL << 8) | (1UL << 7));       // clear bits
  reg_value = reg_value | ( convtime  & 0x07 ) << 7;          // set bits
  TMP117_writeConfig ( reg_value );
}
/*!
    @brief    Set averaging mode
    
    @param    ave  ::TMP117_AVE [NOAVE-AVE8-AVE32-AVE64]
*/
void TMP117_setAveraging ( TMP117_AVE ave ) {
  uint16_t reg_value = TMP117_readConfig ();
  reg_value &= ~((1UL << 6) | (1UL << 5) );       // clear bits
  reg_value = reg_value | ( ave & 0x03 ) << 5;          // set bits
  TMP117_writeConfig ( reg_value );
}

/*!
    @brief    Set offset temperature
    
    @param    double  target offset temperature  in the range of ±256°C  
*/
void      TMP117_setOffsetTemperature ( double offset ) {
  int16_t offset_temp_value = (uint16_t)(offset / TMP117_RESOLUTION);
  TMP117_i2cWrite2B (TMP117_REG_TEMPERATURE_OFFSET , offset_temp_value);
}

/*!
    @brief    Set target temperature for calibration purpose
    
    @param    double  target temperature to calibrate to in the range of ±256°C  
*/
void TMP117_setTargetTemperature ( double target ) {
  double actual_temp = TMP117_getTemperature ( );
  double delta_temp =  target - actual_temp;
  TMP117_setOffsetTemperature ( delta_temp );
}

/*!
    @brief    Read configuration register and handle events.

    @return   uint16_t  read value of the configuration regsiter          
*/
uint16_t  TMP117_readConfig (void) {
  uint16_t reg_value = TMP117_i2cRead2B ( TMP117_REG_CONFIGURATION );
  //uint8_t high_alert = reg_value >> 15 & 1UL;
  //uint8_t low_alert = reg_value >> 14 & 1UL;   
  //uint8_t data_ready = reg_value >> 13 & 1UL;   
  //uint8_t eeprom_busy = reg_value >> 12 & 1UL;   

//  if (data_ready && newDataCallback != NULL)
//    newDataCallback ();

  if (reg_value >> 15 & 1UL) {
    alert_type = HIGHALERT;
  }
  else if (reg_value >> 14 & 1UL) {
    alert_type = LOWALERT;
  }
  else {
    alert_type = NOALERT;
  }
    
  return reg_value;  
}

/*!
    @brief    Returns the recalculated temperature
    
    @return   double  temperature in °C
*/
double TMP117_getTemperature (void) {
  int16_t temp = TMP117_i2cRead2B( TMP117_REG_TEMPERATURE );
  return  (temp * TMP117_RESOLUTION);
}
/*!
    @brief    Get Device Revision 
    
    @return   uint16_t device revision
*/
uint16_t  TMP117_getDeviceRev (void) {
  // read bits [15:12]
  uint16_t raw = TMP117_i2cRead2B( TMP117_REG_DEVICE_ID );
  
  return ( (raw >> 12) & 0x3);
}

/*!
    @brief    Get Device ID (always 0x117)
    
    @return   uint16_t  device ID
*/
uint16_t  TMP117_getDeviceID (void) {
  // read bits [11:0]
  uint16_t raw = TMP117_i2cRead2B( TMP117_REG_DEVICE_ID );
  return (raw & 0x0fff);
}

/*!
    @brief    Returns the information which alert type happend
    
    @return   TMP117_ALERT [NoAlert-HighTempAlert-LowTempAlert]
*/
TMP117_ALERT TMP117_getAlertType ( void ) {
  return alert_type;
}

/*!
    @brief    Returns the content of the offset register in °C
    
    @return   double  offset temperature in °C
*/
double TMP117_getOffsetTemperature (void) {
  int16_t temp = TMP117_i2cRead2B( TMP117_REG_TEMPERATURE_OFFSET );
  return  (temp * TMP117_RESOLUTION);
}

/*!
    @brief    Write data to EEPROM register
    
    @param    data        data to write to the EEPROM
    
    @param    eeprom_nr   represents the EEPROM number [1 - 3] 
*/
void TMP117_writeEEPROM (uint16_t data, uint8_t eeprom_nr) {
  if (!TMP117_EEPROMisBusy()) {
    TMP117_unlockEEPROM();
      switch (eeprom_nr) {
        case 1 : TMP117_i2cWrite2B ( TMP117_REG_EEPROM1, data); break;
        case 2 : TMP117_i2cWrite2B ( TMP117_REG_EEPROM2, data); break;
        case 3 : TMP117_i2cWrite2B ( TMP117_REG_EEPROM3, data); break;
        default: asm("nop");
      }
    TMP117_lockEEPROM();
  }
  else {
    asm("nop");
  }
}

/*!
    @brief    Read data from EEPROM register
    
    @param    eeprom_nr  represents the EEPROM number [1 - 3] 
    
    @return   uint16_t   read EEPROM data
*/
uint16_t  TMP117_readEEPROM (uint8_t eeprom_nr) {
  // read the 48 bit number from the EEPROM
  if (!TMP117_EEPROMisBusy()) {
    uint16_t eeprom_data = 0;
    switch (eeprom_nr) {
        case 1 : eeprom_data = TMP117_i2cRead2B( TMP117_REG_EEPROM1 ); break;
        case 2 : eeprom_data = TMP117_i2cRead2B( TMP117_REG_EEPROM2 ); break;
        case 3 : eeprom_data = TMP117_i2cRead2B( TMP117_REG_EEPROM3 ); break;
        default: asm("nop");
      }
    return eeprom_data;
  }
  else {
    asm("nop");
    return 0x0;
  }
}


/**************************************************************************/
/* ********************* Library internal functions  ******************** */
/**************************************************************************/

/*!
    @brief    Write two bytes (16 bits) to TMP117 I2C sensor
    
    @param    reg  target register
    @param    data data to write
*/
void TMP117_i2cWrite2B (uint8_t reg, uint16_t data){
  
  uint8_t device_addr = TMP117_Address;
  uint8_t buffer[2];
  buffer[0] = (uint8_t)(data&0xff);
  buffer[1] = (uint8_t)(data>>8); 
  SdkEvalI2CWrite(buffer, device_addr, reg, 2);   
  asm("nop");
}

/*!
    @brief    Read two bytes (16 bits) from TMP117 I2C sensor
    
    @param    reg  target register to read from
    
    @return   uint16_t  read data
*/
uint16_t  TMP117_i2cRead2B (uint8_t reg) {  
  
  uint8_t device_addr = TMP117_Address;
  uint8_t buffer[2];
  uint16_t datac;
  SdkEvalI2CRead(buffer, device_addr, reg, 2);
  datac = ((buffer[0] << 8) | buffer[1]); 
  return datac;  
  
}

/*!
    @brief    Write configuration to config register
    
    @param    config_data  configuration
*/
void TMP117_writeConfig (uint16_t config_data) {
  TMP117_i2cWrite2B (TMP117_REG_CONFIGURATION, config_data);
}

/*!
    @brief    Lock EEPROM, write protection
*/
void TMP117_lockEEPROM (void) {
  // clear bit 15
  uint16_t code = 0;
  code &= ~(1UL << 15);
  TMP117_i2cWrite2B ( TMP117_REG_EEPROM_UNLOCK, code );

}

/*!
    @brief    Unlock EEPROM, remove write protection
*/
void TMP117_unlockEEPROM (void) {
  // set bit 15
  uint16_t code = 0;
  code |= 1UL << 15;
  TMP117_i2cWrite2B ( TMP117_REG_EEPROM_UNLOCK, code );

}

/*!
    @brief    States if the EEPROM is busy
    
    @return   Ture if the EEPROM is busy, fals else
*/
uint8_t TMP117_EEPROMisBusy (void) {
  // Bit 14 indicates the busy state of the eeprom
  uint16_t code = TMP117_i2cRead2B ( TMP117_REG_EEPROM_UNLOCK );
  return (uint8_t) ((code >> 14) & 0x01);
}


