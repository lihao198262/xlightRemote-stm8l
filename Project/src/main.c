#include "_global.h"
#include "delay.h"
#include "rf24l01.h"
#include "timer.h"
#include "button.h"
#include "MyMessage.h"
#include "ProtocolParser.h"

/*
Xlight Remoter Program
License: MIT

Auther: Baoshi Sun
Email: bs.sun@datatellit.com, bs.sun@uwaterloo.ca
Github: https://github.com/sunbaoshi1975
Please visit xlight.ca for product details

RF24L01 connector pinout:
GND    VCC
CE     CSN
SCK    MOSI
MISO   IRQ

Connections:
  PB4 -> CE
  PD4 -> CSN (10 buttons), PC6 -> CSN (11 buttons)
  PB5 -> SCK
  PB6 -> MOSI
  PB7 -> MISO
  PD5 -> IRQ

*/

#ifdef TEST
void testio()
{
  GPIO_Init(GPIOC , GPIO_Pin_1 , GPIO_Mode_Out_PP_Low_Slow);
  GPIO_Init(GPIOC , GPIO_Pin_3 , GPIO_Mode_Out_PP_Low_Slow);
  GPIO_Init(GPIOC , GPIO_Pin_5 , GPIO_Mode_Out_PP_Low_Slow);
}
#endif

// Starting Flash block number of backup config
#define BACKUP_CONFIG_BLOCK_NUM         2
#define BACKUP_CONFIG_ADDRESS           (FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS + BACKUP_CONFIG_BLOCK_NUM * FLASH_BLOCK_SIZE)

// RF channel for the sensor net, 0-127
#define RF24_CHANNEL	   		100
#define ADDRESS_WIDTH                   5
#define PLOAD_WIDTH                     32

// Window Watchdog
// Uncomment this line if in debug mode
#define DEBUG_NO_WWDG
#define WWDG_COUNTER                    0x7f
#define WWDG_WINDOW                     0x77

#define MAX_RF_FAILED_TIME              3      // Reset RF module when reach max failed times of sending

// Unique ID for STM8L151x4
#define     UNIQUE_ID_ADDRESS           (0x4926)

// Timeout
#define RTE_TM_CONFIG_MODE              12000  // timeout in config mode, about 120s (12000 * 10ms)

const UC RF24_BASE_RADIO_ID[ADDRESS_WIDTH] = {0x00,0x54,0x49,0x54,0x44};


// Public variables
Config_t gConfig;
DeviceStatus_t gDevStatus[NUM_DEVICES];
MyMessage_t sndMsg, rcvMsg;
uint8_t *psndMsg = (uint8_t *)&sndMsg;
uint8_t *prcvMsg = (uint8_t *)&rcvMsg;
bool gNeedSaveBackup = FALSE;
bool gIsStatusChanged = FALSE;
bool gIsChanged = FALSE;
bool gResetRF = FALSE;
bool gResetNode = FALSE;

uint8_t gDelayedOperation = 0;
uint8_t _uniqueID[UNIQUE_ID_LEN];
uint8_t m_cntRFSendFailed = 0;
uint8_t gSendScenario = 0;
uint8_t gSendDelayTick = 0;

// add favorite function
int8_t gLastFavoriteIndex = -1;
uint8_t gLastFavoriteTick = 255;
// is flash writting
uint8_t flashWritting = 0;

// Moudle variables
bool bPowerOn = FALSE;
uint8_t mutex;
uint8_t oldCurrentDevID = 0;
uint16_t configMode_tick = 0;
void tmrProcess();

bool isIdentityEmpty(const UC *pId, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId[i] > 0) return FALSE; }
  return TRUE;
}

bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId1[i] != pId2[i]) return FALSE; }
  return TRUE;
}

bool isNodeIdRequired()
{
#ifndef ENABLE_SDTM
  if( gConfig.enSDTM ) return FALSE;
  
  return( (IS_NOT_REMOTE_NODEID(CurrentNodeID) && !IS_GROUP_NODEID(CurrentNodeID)) || 
         isIdentityEmpty(CurrentNetworkID, ADDRESS_WIDTH) || isIdentityEqual(CurrentNetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) );
#else
  return FALSE;
#endif  
}

static void clock_init(void)
{
  CLK_DeInit();
  CLK_HSICmd(ENABLE);
  CLK_SYSCLKDivConfig(SYS_CLOCK_DIVIDER);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
  CLK_ClockSecuritySystemEnable();
}

// Initialize Window Watchdog
void wwdg_init() {
#ifndef DEBUG_NO_WWDG  
  WWDG_Init(WWDG_COUNTER, WWDG_WINDOW);
#endif  
}

// Feed the Window Watchdog
void feed_wwdg(void) {
#ifndef DEBUG_NO_WWDG    
  uint8_t cntValue = WWDG_GetCounter() & WWDG_COUNTER;
  if( cntValue < WWDG_WINDOW ) {
    WWDG_SetCounter(WWDG_COUNTER);
  }
#endif
}

/**
  * @brief  configure GPIOs before entering low power
	* @caller lowpower_config
  * @param None
  * @retval None
  */  
void GPIO_LowPower_Config(void)
{
  GPIO_Init(GPIOA, GPIO_Pin_2|GPIO_Pin_4, GPIO_Mode_In_FL_No_IT);
  GPIO_Init(GPIOA, GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);

//#ifdef ENABLE_FLASHLIGHT_LASER
//  GPIO_Init(GPIOC, GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6, GPIO_Mode_Out_PP_Low_Slow);
//#else  
  GPIO_Init(GPIOC, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);
//#endif
  
  //GPIO_Init(GPIOD, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3, GPIO_Mode_In_FL_No_IT);
  //GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_In_FL_No_IT);
  GPIO_Init(GPIOE, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Slow);
  GPIO_Init(GPIOF,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7 ,GPIO_Mode_Out_PP_Low_Slow);
}

// Enter Low Power Mode, which can be woken up by external interupts
void lowpower_config(void) {
  // Set STM8 in low power
  PWR->CSR2 = 0x2;
 
  // Stop Timers
  TIM4_DeInit();
  
  // Set GPIO in low power
  GPIO_LowPower_Config();
  
  // RF24 Chip in low power
  RF24L01_DeInit();
  
  // Stop RTC Source clock
  CLK_RTCClockConfig(CLK_RTCCLKSource_Off, CLK_RTCCLKDiv_1);
  
  CLK_LSICmd(DISABLE);
  while ((CLK->ICKCR & 0x04) != 0x00);
  
  // Stop peripheral clocks
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM5, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_DAC, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_RTC, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_LCD, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_AES, DISABLE);
}

// Resume Normal Mode
void wakeup_config(void) {
  clock_init();
  timer_init();
  RF24L01_init();
  NRF2401_EnableIRQ();
}

int8_t wait_flashflag_status(uint8_t flag,uint8_t status)
{
    uint16_t timeout = 60000;
    while( FLASH_GetFlagStatus(flag)== status && timeout--);
    if(!timeout) 
    {
      //printlog("timeout!");
      return 1;
    }
    return 0;
}

void Flash_ReadBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length) {
  assert_param(IS_FLASH_ADDRESS(Address));
  assert_param(IS_FLASH_ADDRESS(Address+Length));
  
  for( uint16_t i = 0; i < Length; i++ ) {
    Buffer[i] = FLASH_ReadByte(Address+i);
  }
}

bool Flash_WriteBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length) {
  assert_param(IS_FLASH_DATA_EEPROM_ADDRESS(Address));
  assert_param(IS_FLASH_DATA_EEPROM_ADDRESS(Address+Length));
  if(flashWritting == 1)
  {
    //printlog("iswriting");
    return FALSE;
  }
  flashWritting = 1;
  // Init Flash Read & Write
  FLASH_SetProgrammingTime(FLASH_ProgramMode_Standard);

  FLASH_Unlock(FLASH_MemType_Data);
  /* Wait until Data EEPROM area unlocked flag is set*/
  //while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET);
  if(wait_flashflag_status(FLASH_FLAG_DUL,RESET)) return FALSE;
  // Write byte by byte
  bool rc = TRUE;
  uint8_t bytVerify, bytAttmpts;
  for( uint16_t i = 0; i < Length; i++ ) {
    bytAttmpts = 0;
    while(++bytAttmpts <= 3) {
      FLASH_ProgramByte(Address+i, Buffer[i]);
      FLASH_WaitForLastOperation(FLASH_MemType_Data);
      
      // Read and verify the byte we just wrote
      bytVerify = FLASH_ReadByte(Address+i);
      if( bytVerify == Buffer[i] ) break;
    }
    if( bytAttmpts > 3 ) {
      rc = FALSE;
      break;
    }
  }
  FLASH_Lock(FLASH_MemType_Data);
  flashWritting = 0;
  return rc;
}
 
bool Flash_WriteDataBlock(uint16_t nStartBlock, uint8_t *Buffer, uint16_t Length) {
  // Init Flash Read & Write
  if(flashWritting == 1) 
  {
    //printlog("iswriting");
    return FALSE;
  }
  flashWritting = 1;

  FLASH_SetProgrammingTime(FLASH_ProgramMode_Standard);
  FLASH_Unlock(FLASH_MemType_Data);
  //while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET);
  if(wait_flashflag_status(FLASH_FLAG_DUL,RESET)) return FALSE;
  uint8_t WriteBuf[FLASH_BLOCK_SIZE];
  uint16_t nBlockNum = (Length - 1) / FLASH_BLOCK_SIZE + 1;
  for( uint16_t block = nStartBlock; block < nStartBlock + nBlockNum; block++ ) {
    memset(WriteBuf, 0x00, FLASH_BLOCK_SIZE);
    for( uint16_t i = 0; i < FLASH_BLOCK_SIZE; i++ ) {
      WriteBuf[i] = Buffer[(block - nStartBlock) * FLASH_BLOCK_SIZE + i];
    }
#ifdef TEST
  PC3_High;
#endif 
    FLASH_ProgramBlock(block, FLASH_MemType_Data, FLASH_ProgramMode_Standard, WriteBuf);
    // very important( FLASH_WaitForLastOperation(FLASH_MemType_Data) not usefull,cause program dead)
    if(wait_flashflag_status(FLASH_FLAG_EOP,RESET)) return FALSE;
#ifdef TEST
  PC3_Low;
#endif   
  }
  FLASH_Lock(FLASH_MemType_Data);

  flashWritting = 0;
  return TRUE;
}

uint8_t *Read_UniqueID(uint8_t *UniqueID, uint16_t Length)  
{
  Flash_ReadBuf(UNIQUE_ID_ADDRESS, UniqueID, Length);
  return UniqueID;
}

/*// Save config to Flash
void SaveConfig()
{
#ifndef ENABLE_SDTM
  if( gIsChanged ) {
    Flash_WriteBuf(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    gIsChanged = FALSE;
  }
#endif  
}*/

// Save config to Flash
void SaveBackupConfig()
{
  if( gNeedSaveBackup ) {
    // Overwrite entire config FLASH
    if(Flash_WriteDataBlock(BACKUP_CONFIG_BLOCK_NUM, (uint8_t *)&gConfig, sizeof(gConfig)))
    {
      gNeedSaveBackup = FALSE;
    }
  }
}

// Save status to Flash
void SaveStatusData()
{
    // Skip the first byte (version)
    uint8_t pData[50] = {0};
    uint16_t nLen = (uint16_t)(&(gConfig.nodeID)) - (uint16_t)(&gConfig);
    memcpy(pData, (uint8_t *)&gConfig, nLen);
    Flash_WriteBuf(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS + 1, pData + 1, nLen - 1);
    gIsStatusChanged = FALSE;
}

// Save config to Flash
void SaveConfig()
{
  if( gIsChanged ) {
    // Overwrite entire config FLASH
    if(Flash_WriteDataBlock(0, (uint8_t *)&gConfig, sizeof(gConfig)))
    {
      gIsStatusChanged = FALSE;
      gIsChanged = FALSE;
      gNeedSaveBackup = TRUE;
      return;
    }
  }

  if( gIsStatusChanged ) {
    // Overwrite only Static & status parameters (the first part of config FLASH)
    SaveStatusData();
  }  
}

// Init Device Status
void InitDeviceStatus()
{
  DEVST_Present(0) = 0;
  DEVST_OnOff(0) = 0;
  DEVST_Bright(0) = 50;
  DEVST_WarmCold(0) = CT_MIN_VALUE;
  for(UC i = 1; i < NUM_DEVICES; i++ ) {
    gDevStatus[i] = gDevStatus[0];
  }
}

bool IsConfigInvalid() {
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT 
       || !IS_VALID_REMOTE(gConfig.type)  || gConfig.nodeID == 0
       || gConfig.rfPowerLevel > RF24_PA_MAX || gConfig.rfChannel > 127 || gConfig.rfDataRate > RF24_250KBPS );
}

// Load config from Flash
void LoadConfig()
{
    // Load the most recent settings from FLASH
    Flash_ReadBuf(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    if( IsConfigInvalid() ) {
      Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
      if( IsConfigInvalid() ) {
        memset(&gConfig, 0x00, sizeof(gConfig));
        gConfig.version = XLA_VERSION;
        gConfig.indDevice = 0;
        gConfig.present = 0;
        gConfig.inPresentation = 0;
        gConfig.enSDTM = 0;
#ifdef BATCH_TEST
        gConfig.rptTimes = 3;
#else
        gConfig.rptTimes = 1;
#endif
        gConfig.type = remotetypRFStandard;
        gConfig.rfChannel = RF24_CHANNEL;
        gConfig.rfPowerLevel = RF24_PA_MAX;
        gConfig.rfDataRate = RF24_250KBPS;      
        memcpy(CurrentNetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
        //sprintf(gConfig.Organization, "%s", XLA_ORGANIZATION);
        //sprintf(gConfig.ProductName, "%s", XLA_PRODUCT_NAME);

        // Set device info
        NodeID(0) = BASESERVICE_ADDRESS;       // NODEID_MIN_REMOTE; BASESERVICE_ADDRESS; NODEID_DUMMY
        DeviceID(0) = NODEID_MAINDEVICE;
        DeviceType(0) = devtypDummy;
        gConfig.devItem[1] = gConfig.devItem[0];
        gConfig.devItem[2] = gConfig.devItem[0];
        gConfig.devItem[3] = gConfig.devItem[0];
        
#ifdef MAIN_LAMP_RGBW
        gConfig.fnScenario[0].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[0].hue.bmRing = 0;
        gConfig.fnScenario[0].hue.BR = BTN_FN1_BR;
        gConfig.fnScenario[0].hue.CCT = BTN_FN1_W;
        gConfig.fnScenario[0].hue.R = BTN_FN1_R;
        gConfig.fnScenario[0].hue.G = BTN_FN1_G;
        gConfig.fnScenario[0].hue.B = BTN_FN1_B;        
        gConfig.fnScenario[0].scenario = BTN_FN1_SC;
        gConfig.fnScenario[0].effect = FILTER_SP_EF_NONE;

        gConfig.fnScenario[1].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[1].hue.bmRing = 0;
        gConfig.fnScenario[1].hue.BR = BTN_FN2_BR;
        gConfig.fnScenario[1].hue.CCT = BTN_FN2_W;
        gConfig.fnScenario[1].hue.R = BTN_FN2_R;
        gConfig.fnScenario[1].hue.G = BTN_FN2_G;
        gConfig.fnScenario[1].hue.B = BTN_FN2_B;        
        gConfig.fnScenario[1].scenario = BTN_FN2_SC;
        gConfig.fnScenario[1].effect = FILTER_SP_EF_NONE;

        gConfig.fnScenario[2].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[2].hue.bmRing = 0;
        gConfig.fnScenario[2].hue.BR = BTN_FN3_BR;
        gConfig.fnScenario[2].hue.CCT = BTN_FN3_W;
        gConfig.fnScenario[2].hue.R = BTN_FN3_R;
        gConfig.fnScenario[2].hue.G = BTN_FN3_G;
        gConfig.fnScenario[2].hue.B = BTN_FN3_B;
        gConfig.fnScenario[2].scenario = BTN_FN3_SC;
        gConfig.fnScenario[2].effect = FILTER_SP_EF_NONE;

        gConfig.fnScenario[3].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[3].hue.bmRing = 0;
        gConfig.fnScenario[3].hue.BR = BTN_FN4_BR;
        gConfig.fnScenario[3].hue.CCT = BTN_FN4_W;
        gConfig.fnScenario[3].hue.R = BTN_FN4_R;
        gConfig.fnScenario[3].hue.G = BTN_FN4_G;
        gConfig.fnScenario[3].hue.B = BTN_FN4_B;
        gConfig.fnScenario[3].scenario = BTN_FN4_SC;
        gConfig.fnScenario[3].effect = FILTER_SP_EF_NONE;

        gConfig.fnScenario[4].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[4].hue.bmRing = 0;
        gConfig.fnScenario[4].hue.BR = BTN_FN5_BR;
        gConfig.fnScenario[4].hue.CCT = BTN_FN5_W;
        gConfig.fnScenario[4].hue.R = BTN_FN5_R;
        gConfig.fnScenario[4].hue.G = BTN_FN5_G;
        gConfig.fnScenario[4].hue.B = BTN_FN5_B;
        gConfig.fnScenario[4].scenario = BTN_FN5_SC;
        gConfig.fnScenario[4].effect = FILTER_SP_EF_NONE;

        gConfig.fnScenario[5].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[5].hue.bmRing = 0;
        gConfig.fnScenario[5].hue.BR = BTN_FN6_BR;
        gConfig.fnScenario[5].hue.CCT = BTN_FN6_W;
        gConfig.fnScenario[5].hue.R = BTN_FN6_R;
        gConfig.fnScenario[5].hue.G = BTN_FN6_G;
        gConfig.fnScenario[5].hue.B = BTN_FN6_B;
        gConfig.fnScenario[5].scenario = BTN_FN6_SC;
        gConfig.fnScenario[5].effect = FILTER_SP_EF_BREATH;

        gConfig.fnScenario[6].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[6].hue.bmRing = 0;
        gConfig.fnScenario[6].hue.BR = BTN_FN7_BR;
        gConfig.fnScenario[6].hue.CCT = BTN_FN7_W;
        gConfig.fnScenario[6].hue.R = BTN_FN7_R;
        gConfig.fnScenario[6].hue.G = BTN_FN7_G;
        gConfig.fnScenario[6].hue.B = BTN_FN7_B;
        gConfig.fnScenario[6].scenario = BTN_FN7_SC;
        gConfig.fnScenario[6].effect = FILTER_SP_EF_FLORID;
#else
        gConfig.fnScenario[0].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[0].hue.bmRing = 0;
        gConfig.fnScenario[0].hue.BR = BTN_FN1_BR;
        gConfig.fnScenario[0].hue.CCT = BTN_FN1_CCT;
        gConfig.fnScenario[0].scenario = BTN_FN1_SC;
        gConfig.fnScenario[0].effect = FILTER_SP_EF_NONE;
        
        gConfig.fnScenario[1].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[1].hue.bmRing = 0;
        gConfig.fnScenario[1].hue.BR = BTN_FN2_BR;
        gConfig.fnScenario[1].hue.CCT = BTN_FN2_CCT;
        gConfig.fnScenario[1].scenario = BTN_FN2_SC;
        gConfig.fnScenario[1].effect = FILTER_SP_EF_NONE;
        
        gConfig.fnScenario[2].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[2].hue.bmRing = 0;
        gConfig.fnScenario[2].hue.BR = BTN_FN3_BR;
        gConfig.fnScenario[2].hue.CCT = BTN_FN3_CCT;
        gConfig.fnScenario[2].scenario = BTN_FN3_SC;
        gConfig.fnScenario[2].effect = FILTER_SP_EF_NONE;
        
        gConfig.fnScenario[3].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[3].hue.bmRing = 0;
        gConfig.fnScenario[3].hue.BR = BTN_FN4_BR;
        gConfig.fnScenario[3].hue.CCT = BTN_FN4_CCT;
        gConfig.fnScenario[3].scenario = BTN_FN4_SC;
        gConfig.fnScenario[3].effect = FILTER_SP_EF_NONE;

        gConfig.fnScenario[4].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[4].hue.bmRing = 0;
        gConfig.fnScenario[4].hue.BR = BTN_FN5_BR;
        gConfig.fnScenario[4].hue.CCT = BTN_FN5_CCT;
        gConfig.fnScenario[4].scenario = BTN_FN5_SC;
        gConfig.fnScenario[4].effect = FILTER_SP_EF_NONE;

        gConfig.fnScenario[5].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[5].hue.bmRing = 0;
        gConfig.fnScenario[5].hue.BR = BTN_FN6_BR;
        gConfig.fnScenario[5].hue.CCT = BTN_FN6_CCT;
        gConfig.fnScenario[5].scenario = BTN_FN6_SC;
        gConfig.fnScenario[5].effect = FILTER_SP_EF_BREATH;

        gConfig.fnScenario[6].hue.State = DEVICE_SW_ON;
        gConfig.fnScenario[6].hue.bmRing = 0;
        gConfig.fnScenario[6].hue.BR = BTN_FN7_BR;
        gConfig.fnScenario[6].hue.CCT = BTN_FN7_CCT;
        gConfig.fnScenario[6].scenario = BTN_FN7_SC;
        gConfig.fnScenario[6].effect = FILTER_SP_EF_FAST_BREATH;
#endif
      }
      gIsChanged = TRUE;
    }
    else {
      uint8_t bytVersion;
      Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, (uint8_t *)&bytVersion, sizeof(bytVersion));
      if( bytVersion != gConfig.version ) gNeedSaveBackup = TRUE;
    }
        // Session time parameters
    gConfig.indDevice = 0;
    gConfig.inConfigMode = 0;
    gConfig.inPresentation = 0;
    oldCurrentDevID = gConfig.indDevice;
  
#ifdef CLASS_ROOM_TYPE
    
    gConfig.relayKey.deviceID = 129;
    gConfig.relayKey.subDevID = 4;
    gConfig.relayKey.keys[0] = '1';
    gConfig.relayKey.keys[1] = 0;
    gConfig.relayKey.state = 0;
    
     // Set Devices
    gConfig.devItem[0].deviceID = 255;
    gConfig.devItem[0].subDevID = 0;
    gConfig.devItem[1] = gConfig.devItem[0];
    gConfig.devItem[2] = gConfig.devItem[0];
    gConfig.devItem[3] = gConfig.devItem[0];
    // blackboard light
    gConfig.devItem[1].deviceID = 255;
    gConfig.devItem[1].subDevID = 1;
    // classroom light
    gConfig.devItem[2].deviceID = 255;
    gConfig.devItem[2].subDevID = 14;
    // curtain
    gConfig.devItem[3].deviceID = 129;
    gConfig.devItem[3].subDevID = 8;
    
    // Set Fn
    /// F1 all light 90,5500
    gConfig.fnScenario[0].bmDevice = 0x01;
    gConfig.fnScenario[0].scenario = 65;
    gConfig.fnScenario[0].hue.State = 1;
    gConfig.fnScenario[0].hue.bmRing = 0;
    gConfig.fnScenario[0].hue.BR = 90;
    gConfig.fnScenario[0].hue.CCT = 5500;
    /// F2 classroomlight 90,3000
    gConfig.fnScenario[1].bmDevice = 0x04;
    gConfig.fnScenario[1].scenario = 66;
    gConfig.fnScenario[1].hue.State = 1;
    gConfig.fnScenario[1].hue.bmRing = 0;
    gConfig.fnScenario[1].hue.BR = 90;
    gConfig.fnScenario[1].hue.CCT = 3000;
    /// F3 blackboard light off
    gConfig.fnScenario[2].bmDevice = 0x02;
    gConfig.fnScenario[2].scenario = 67;
    gConfig.fnScenario[2].hue.State = 0;
    gConfig.fnScenario[2].hue.bmRing = 0;
    /// F4 classroomlight 50,5500
    gConfig.fnScenario[3].bmDevice = 0x04;
    gConfig.fnScenario[3].scenario = 68;
    gConfig.fnScenario[3].hue.State = 1;
    gConfig.fnScenario[3].hue.bmRing = 0;
    gConfig.fnScenario[3].hue.BR = 50;
    gConfig.fnScenario[3].hue.CCT = 3000;
    /// F5 classroomlight light off
    gConfig.fnScenario[4].bmDevice = 0x04;
    gConfig.fnScenario[4].scenario = 69;
    gConfig.fnScenario[4].hue.State = 0;
    gConfig.fnScenario[4].hue.bmRing = 0;
    /// F6 classroomlight light toggle
    gConfig.fnScenario[5].bmDevice = 0x04;
    gConfig.fnScenario[5].scenario = 71;
    gConfig.fnScenario[5].hue.State = 2;
    gConfig.fnScenario[5].hue.bmRing = 0;
    /// F7 blackboard light toggle
    gConfig.fnScenario[6].bmDevice = 0x02;
    gConfig.fnScenario[6].scenario = 72;
    gConfig.fnScenario[6].hue.State = 2;
    gConfig.fnScenario[6].hue.bmRing = 0;
    /*/// F8 curtain off
    gConfig.fnScenario[5].bmDevice = 0x08;
    gConfig.fnScenario[5].scenario = 70;
    gConfig.fnScenario[5].hue.State = 0;*/

#endif

#ifdef HOME_VERSION
       // Set Devices
    gConfig.devItem[0].deviceID = 254;
    gConfig.devItem[0].subDevID = 0;
        // Set Fn
    /// F1 light 100,5500
    gConfig.fnScenario[0].bmDevice = 0x01;
    gConfig.fnScenario[0].scenario = 0;
    gConfig.fnScenario[0].hue.State = 1;
    gConfig.fnScenario[0].hue.bmRing = 0;
    gConfig.fnScenario[0].hue.BR = 100;
    gConfig.fnScenario[0].hue.CCT = 5500;
    /// F2 light 20
    gConfig.fnScenario[1].bmDevice = 0x01;
    gConfig.fnScenario[1].scenario = 0;
    gConfig.fnScenario[1].hue.State = 1;
    gConfig.fnScenario[1].hue.bmRing = 0;
    gConfig.fnScenario[1].hue.BR = 20;
    /// F3 light 60，4000
    gConfig.fnScenario[2].bmDevice = 0x01;
    gConfig.fnScenario[2].scenario = 0;
    gConfig.fnScenario[2].hue.State = 1;
    gConfig.fnScenario[2].hue.bmRing = 0;
    gConfig.fnScenario[2].hue.BR = 60;
    gConfig.fnScenario[2].hue.CCT = 4000;
    /// F4 light 10,3000
    gConfig.fnScenario[3].bmDevice = 0x01;
    gConfig.fnScenario[3].scenario = 0;
    gConfig.fnScenario[3].hue.State = 1;
    gConfig.fnScenario[3].hue.bmRing = 0;
    gConfig.fnScenario[3].hue.BR = 10;
    gConfig.fnScenario[3].hue.CCT = 3000;
#endif
}

void UpdateNodeAddress(uint8_t _tx) {
  memcpy(rx_addr, CurrentNetworkID, ADDRESS_WIDTH);
  rx_addr[0] = CurrentNodeID;
  memcpy(tx_addr, CurrentNetworkID, ADDRESS_WIDTH);
  
  if( _tx == NODEID_RF_SCANNER ) {
    tx_addr[0] = NODEID_RF_SCANNER;
  } else {  
#ifdef BATCH_TEST
    tx_addr[0] = BROADCAST_ADDRESS;
#else
#ifdef ENABLE_SDTM
    tx_addr[0] = CurrentDeviceID;
#else
    if( gConfig.enSDTM ) {
      tx_addr[0] = CurrentDeviceID;
    } else {
      tx_addr[0] = (isNodeIdRequired() ? BASESERVICE_ADDRESS : NODEID_GATEWAY);
    }
#endif

#endif
  }
  RF24L01_setup(gConfig.rfChannel, gConfig.rfDataRate, gConfig.rfPowerLevel, BROADCAST_ADDRESS);     // With openning the boardcast pipe
}  

bool NeedUpdateRFAddress(uint8_t _dest) {
  bool rc = FALSE;
  if( sndMsg.header.destination == NODEID_RF_SCANNER && tx_addr[0] != NODEID_RF_SCANNER ) {
    UpdateNodeAddress(NODEID_RF_SCANNER);
    rc = TRUE;
  } else if( sndMsg.header.destination != NODEID_RF_SCANNER && tx_addr[0] != NODEID_GATEWAY ) {
    UpdateNodeAddress(NODEID_GATEWAY);
    rc = TRUE;
  }
  return rc;
}

void EraseCurrentDeviceInfo() {
#ifndef ENABLE_SDTM
  gDelayedOperation = DELAY_OP_ERASEFLASH;
  CurrentNodeID = BASESERVICE_ADDRESS;
  CurrentDeviceID = NODEID_MAINDEVICE;
  CurrentDeviceType = devtypMRing3;
  CurrentDevicePresent = 0;
  gConfig.enSDTM = 0;
  memcpy(CurrentNetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
  gIsChanged = TRUE;
  SaveConfig();
#endif
}

void ToggleSDTM() {
#ifndef ENABLE_SDTM  
  gConfig.enSDTM = 1 - gConfig.enSDTM;
  gIsChanged = TRUE;
  SaveConfig();
  // Soft reset
  WWDG->CR = 0x80;
#endif  
}

void SetConfigMode(bool _sw, uint8_t _devIndex) {
  configMode_tick = 0;
  if( !gConfig.inConfigMode ) oldCurrentDevID = gConfig.indDevice;
  gConfig.inConfigMode = _sw;
  
  // May use ChangeCurrentDevice() later
  if( _devIndex == 255 ) gConfig.indDevice = oldCurrentDevID;
  else if( _devIndex < NUM_DEVICES ) gConfig.indDevice = _devIndex;
}

bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( mutex > 0 ) return TRUE;
  }
  return FALSE;
}

// reset rf
void ResetRFModule()
{
  if(gResetRF)
  {
    RF24L01_init();
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    gResetRF=FALSE;
  }
}


// Send message and switch back to receive mode
bool SendMyMessage() {
  if( bMsgReady ) {
    
    // Change tx destination if necessary
    NeedUpdateRFAddress(sndMsg.header.destination);
    
    uint8_t lv_tried = 0;
    uint16_t delay;
    while (lv_tried++ <= gConfig.rptTimes ) {
      mutex = 0;
      RF24L01_set_mode_TX();
      RF24L01_write_payload(psndMsg, PLOAD_WIDTH);

      WaitMutex(0x1FFFF);
      if (mutex == 1) {
        m_cntRFSendFailed = 0;
        break; // sent sccessfully
      } else if( m_cntRFSendFailed++ > MAX_RF_FAILED_TIME ) {
        // Reset RF module
        m_cntRFSendFailed = 0;
        // RF24 Chip in low power
        RF24L01_DeInit();
        delay = 0x1FFF;
        while(delay--)feed_wwdg();
        RF24L01_init();
        NRF2401_EnableIRQ();
        UpdateNodeAddress(NODEID_GATEWAY);
        continue;
      }
      
      //The transmission failed, Notes: mutex == 2 doesn't mean failed
      //It happens when rx address defers from tx address
      //asm("nop"); //Place a breakpoint here to see memory
      // Repeat the message if necessary
      uint16_t delay = 0xFFF;
      while(delay--)feed_wwdg();
    }
    
    // Switch back to receive mode
    bMsgReady = 0;
    RF24L01_set_mode_RX();
  }

  return(mutex > 0);
}

bool SayHelloToDevice(bool infinate) {
  uint8_t _count = 0;
  UpdateNodeAddress(NODEID_GATEWAY);
  while(1) {
    ////////////rfscanner process///////////////////////////////
    ProcessOutputCfgMsg(); 
    SendMyMessage();
    ResetRFModule();
    SaveConfig();
    // Save config into backup area
    SaveBackupConfig();
    ////////////rfscanner process/////////////////////////////// 
    if( _count++ == 0 ) {
      if( isNodeIdRequired() ) {
        Msg_RequestNodeID();
      } else {
        Msg_Presentation();
      }
      
      if( SendMyMessage() ) break;
      if( !infinate ) return FALSE;
    }

    // Feed the Watchdog
    feed_wwdg();
    
    // Failed or Timeout, then repeat init-step
    delay_ms(500);
    _count %= 20;  // Every 10 seconds
  }
  
  return TRUE;
}

// ToDo: change to async operation and blink LED during pairing
uint8_t ChangeCurrentDevice(uint8_t _newDev) {
  if( _newDev >= NUM_DEVICES ) _newDev = 0;
  
  uint8_t currentDev = gConfig.indDevice;
  if( currentDev != _newDev ) {
    SelectDeviceLED(_newDev);
    gConfig.indDevice = _newDev;
    UpdateNodeAddress(NODEID_GATEWAY);
    if( !SayHelloToDevice(FALSE) ) {
      // Switch back to prevoius device
      SelectDeviceLED(currentDev);
      gConfig.indDevice = currentDev;
      UpdateNodeAddress(NODEID_GATEWAY);
    }
  }
  
  return gConfig.indDevice;
}

// Change LED or Laser to indecate execution of specific operation
void OperationIndicator() {
  static uint16_t tick = 0;
  uint8_t test, step;
  test = gDelayedOperation & 0xF0;
  if( test == DELAY_OP_ERASEFLASH ) {
    // LED fast blink 5 times
    step = gDelayedOperation - DELAY_OP_ERASEFLASH;
    if( step >= 10 ) {
      gDelayedOperation = 0;    // Finished
      SetFlashlight(DEVICE_SW_OFF);
      // Soft reset
      WWDG->CR = 0x80;
    } else {
      if( step == 0 ) {
        SetFlashlight(DEVICE_SW_ON);
        tick = 0;
        gDelayedOperation++;
      }
      if( ++tick > 0x40FF ) {
        tick = 0;
        SetFlashlight(step % 2 ? DEVICE_SW_OFF : DEVICE_SW_ON);
        gDelayedOperation++;
      }
    }
  }
  else if( test == DELAY_OP_PAIRED ) {
    // LED slow blink 1 time and fast 3 times
    step = gDelayedOperation - DELAY_OP_PAIRED;
    if( step >= 8 ) {
      gDelayedOperation = 0;    // Finished
      SetFlashlight(DEVICE_SW_OFF);
    } else {
      if( step == 0 ) {
        SetFlashlight(DEVICE_SW_ON);
        tick = 0;
        gDelayedOperation++;
      }
      ++tick;
      if( (step >= 2 && tick > 0x40FF) || tick > 0xBFFF ) {
        tick = 0;
        SetFlashlight(step % 2 ? DEVICE_SW_OFF : DEVICE_SW_ON);
        gDelayedOperation++;
      }
    }
  }
  else if( test == DELAY_OP_CONNECTED ) {
    // LED fast 3 times
    step = gDelayedOperation - DELAY_OP_CONNECTED;
    if( step >= 6 ) {
      gDelayedOperation = 0;    // Finished
      SetFlashlight(DEVICE_SW_OFF);
    } else {
      if( step == 0 ) {
        SetFlashlight(DEVICE_SW_ON);
        tick = 0;
        gDelayedOperation++;
      }
      if( ++tick > 0x40FF ) {
        tick = 0;
        SetFlashlight(step % 2 ? DEVICE_SW_OFF : DEVICE_SW_ON);
        gDelayedOperation++;
      }
    }
  }
  else if( test == DELAY_OP_PPTMODE_ON ) {
    // Laser slow 2 times
    step = gDelayedOperation - DELAY_OP_PPTMODE_ON;
    if( step >= 4 ) {
      gDelayedOperation = 0;    // Finished
      SetLasterBeam(DEVICE_SW_OFF);
    } else {
      if( step == 0 ) {
        SetLasterBeam(DEVICE_SW_ON);
        tick = 0;
        gDelayedOperation++;
      }
      if( ++tick > 0xBFFF ) {
        tick = 0;
        SetLasterBeam(step % 2 ? DEVICE_SW_OFF : DEVICE_SW_ON);
        gDelayedOperation++;
      }
    }
  }
  else if( test == DELAY_OP_PPTMODE_OFF ) {
    // Laser fast 3 times
    step = gDelayedOperation - DELAY_OP_PPTMODE_OFF;
    if( step >= 6 ) {
      gDelayedOperation = 0;    // Finished
      SetLasterBeam(DEVICE_SW_OFF);
    } else {
      if( step == 0 ) {
        SetLasterBeam(DEVICE_SW_ON);
        tick = 0;
        gDelayedOperation++;
      }
      if( ++tick > 0x40FF ) {
        tick = 0;
        SetLasterBeam(step % 2 ? DEVICE_SW_OFF : DEVICE_SW_ON);
        gDelayedOperation++;
      }
    }
  }
}

int main( void ) {

  // Init clock, timer and button
  clock_init();
  timer_init();
  button_init();

  //GPIO_Init(GPIOC, (GPIO_Pin_0 | GPIO_Pin_2), GPIO_Mode_Out_PP_Low_Fast);
  //SetFlashlight(DEVICE_SW_ON);
  //SetLasterBeam(DEVICE_SW_ON);
  //GPIO_WriteBit(GPIOC, GPIO_Pin_0, SET);
  //GPIO_WriteBit(GPIOC, GPIO_Pin_2, SET);
  //while (1);

  // Go on only if NRF chip is presented
  RF24L01_init();
  while(!NRF24L01_Check());
  
  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();

  // Init Device Status Buffer
  InitDeviceStatus();

  // Blink LED to indicate starting
  SetLasterBeam(DEVICE_SW_OFF);
  SetFlashlight(DEVICE_SW_OFF);
  LED_Blink(TRUE, FALSE);
  LED_Blink(TRUE, FALSE);
 
  // NRF_IRQ
  NRF2401_EnableIRQ();
  
#ifdef BATCH_TEST
  // batch test mode
  gConfig.indDevice = 0;
  CurrentNodeID = NODEID_MIN_REMOTE;
  CurrentDeviceID = BROADCAST_ADDRESS;
  memcpy(CurrentNetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
  UpdateNodeAddress(NODEID_GATEWAY);
#else
  // Must establish connection firstly
#ifdef ENABLE_SDTM
  gConfig.indDevice = 0;
  CurrentNodeID = NODEID_MIN_REMOTE;
  CurrentDeviceID = BASESERVICE_ADDRESS;
  memcpy(CurrentNetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
  UpdateNodeAddress(NODEID_GATEWAY);
#else
  if( gConfig.enSDTM ) {
    gConfig.indDevice = 0;
    CurrentNodeID = NODEID_MIN_REMOTE;
    CurrentDeviceID = BASESERVICE_ADDRESS;
    memcpy(CurrentNetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
    UpdateNodeAddress(NODEID_GATEWAY);
  } else {
    SayHelloToDevice(TRUE);
  }
#endif
  
#endif



  // Init Watchdog
  wwdg_init();
  
  // Set PowerOn flag
  bPowerOn = TRUE;
  TIM4_10ms_handler = tmrProcess;
#ifdef TEST
   testio();
#endif  
  while (1) {
    
    // Feed the Watchdog
    feed_wwdg();
    
    ////////////rfscanner process///////////////////////////////
    ProcessOutputCfgMsg(); 
    // reset rf
    ResetRFModule();
    if(gResetNode)
    {
      gResetNode = FALSE;
#ifndef ENABLE_SDTM
      if( !gConfig.enSDTM ) {
        SayHelloToDevice(TRUE);
      }
#endif
    }
    ////////////rfscanner process/////////////////////////////// 
    
    // Send message if ready
    SendMyMessage();
    // Save Config if Changed
    SaveConfig(); 
    // Save config into backup area
#ifdef TEST
  PC5_High;
#endif  
    SaveBackupConfig();
#ifdef TEST
  PC5_Low;
#endif   
    // Operation Result Indicator
    if( gDelayedOperation > 0 ) OperationIndicator();
    
    // Enter Low Power Mode
    if( tmrIdleDuration > TIMEOUT_IDLE && !isNodeIdRequired() && !gConfig.inConfigMode ) {
      tmrIdleDuration = 0;
      lowpower_config();
      bPowerOn = FALSE;
      halt();
    } else if( !bPowerOn ) {
      // Wakeup
      bPowerOn = TRUE;
      wakeup_config();
      // REQ device status
      //delay_ms(10);
      //Msg_RequestDeviceStatus();
    }
  }
}

// Execute timer operations
void tmrProcess() {
  if( gConfig.inConfigMode ) {
    if( configMode_tick++ > RTE_TM_CONFIG_MODE ) {
      SetConfigMode(FALSE, oldCurrentDevID);
    }
  }
  
  if( gSendScenario > 0 ) {
    gSendDelayTick++;
    if( gSendDelayTick > 5 && !bMsgReady ) {
      Msg_DevScenario(gSendScenario);
      gSendScenario = 0;
      gSendDelayTick = 0;
    }
  }
#ifdef HOME_VERSION
  if(gLastFavoriteTick != 255 && gLastFavoriteTick < MAXFAVORITE_INTERVAL)
    gLastFavoriteTick++;
#endif
}

void RF24L01_IRQ_Handler() {
  tmrIdleDuration = 0;
  if(RF24L01_is_data_available()) {
    //Packet was received
    RF24L01_clear_interrupts();
    RF24L01_read_payload(prcvMsg, PLOAD_WIDTH);
    bMsgReady = ParseProtocol();
    return;
  }
 
  uint8_t sent_info;
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    RF24L01_clear_interrupts();
    mutex = sent_info; 
    return;
  }

   RF24L01_clear_interrupts();
}