#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <stm8l15x.h> //Required for the stdint typedefs
#include "stdio.h"
#include "string.h"
#include "stm8l15x_conf.h"

// Simple Direct Test
// Uncomment this line to work in Simple Direct Test Mode
//#define ENABLE_SDTM

// Button Layout
// Uncomment this line if PCB has 10 buttons
//#define PCB_10_BUTTONS

// Config Flashlight and Laser
// Uncomment this line if need Flashlight or Laser Pen
//#define ENABLE_FLASHLIGHT_LASER

// Uncomment this line if need Presentation Mode
//#define ENABLE_PRESENTATION_MODE

#define BATCH_TEST
//#define HOME_VERSION
//#ifdef HOME_VERSION
//#define ENABLE_SDTM
//#endif
#define CLASS_ROOM_TYPE
//#define MAIN_LAMP_RGBW

/* Exported types ------------------------------------------------------------*/
// Common Data Type
#define UC                        uint8_t
#define US                        uint16_t
#define UL                        uint32_t
#define SHORT                     int16_t
#define LONG                      int32_t

// Switch value for set power command
#define DEVICE_SW_OFF               0       // Turn Off
#define DEVICE_SW_ON                1       // Turn On
#define DEVICE_SW_TOGGLE            2       // Toggle

// Update operator for set brightness & CCT command
#define OPERATOR_SET                0
#define OPERATOR_ADD                1
#define OPERATOR_SUB                2
#define OPERATOR_MUL                3
#define OPERATOR_DIV                4

// Filter (special effect)
#define FILTER_SP_EF_NONE           0
#define FILTER_SP_EF_BREATH         1       // Normal breathing light
#define FILTER_SP_EF_FAST_BREATH    2       // Fast breathing light
#define FILTER_SP_EF_FLORID         3       // Randomly altering color
#define FILTER_SP_EF_FAST_FLORID    4       // Fast randomly altering color

// Node type
#define NODE_TYP_GW               'g'
#define NODE_TYP_LAMP             'l'
#define NODE_TYP_REMOTE           'r'
#define NODE_TYP_SYSTEM           's'
#define NODE_TYP_THIRDPARTY       't'

// NodeID Convention
#define NODEID_GATEWAY          0
#define NODEID_MAINDEVICE       1
#define NODEID_MIN_DEVCIE       8
#define NODEID_MAX_DEVCIE       63
#define NODEID_MIN_REMOTE       64
#define NODEID_MAX_REMOTE       127
#define NODEID_PROJECTOR        128
#define NODEID_KEYSIMULATOR     129
#define NODEID_SUPERSENSOR      130
#define NODEID_SMARTPHONE       139
#define NODEID_MIN_GROUP        192
#define NODEID_MAX_GROUP        223
#define NODEID_RF_SCANNER       250
#define NODEID_DUMMY            255
#define BASESERVICE_ADDRESS     0xFE
#define BROADCAST_ADDRESS       0xFF

#define BR_MIN_VALUE            1
#define CT_MIN_VALUE            2700
#define CT_MAX_VALUE            6500
#define CT_SCOPE                38    
#define CT_STEP                 ((CT_MAX_VALUE-CT_MIN_VALUE)/10)

#define UNIQUE_ID_LEN           8
#define NUM_DEVICES             4
#define NUM_RELAY_KEYS          4

#define DELAY_OP_ERASEFLASH     0x10
#define DELAY_OP_PAIRED         0x20
#define DELAY_OP_CONNECTED      0x30
#define DELAY_OP_PPTMODE_ON     0x40
#define DELAY_OP_PPTMODE_OFF    0x50

#ifdef HOME_VERSION
#define SLEEP_TIME   15
#define NUM_FAVORITE 2  

#define SECOND_UNIT  1
#define MINUTE_UNIT  2
#define HOUR_UNIT    3
#endif

// Device (lamp) type
typedef enum
{
  devtypUnknown = 0,
  // Color ring - Rainbow
  devtypCRing3,
  devtypCRing2,
  devtypCBar,
  devtypCFrame,
  devtypCWave,
  devtypCRing1 = 31,

  // White ring - Sunny
  devtypWRing3 = 32,
  devtypWRing2,
  devtypWBar,
  devtypWFrame,
  devtypWWave,
  devtypWSquare60,      // 60 * 60
  devtypWPanel120_30,   // 120 * 30
  devtypWBlackboard,    // Blackboard lamp
  devtypWRing1 = 95,

  // Color & Motion ring - Mirage
  devtypMRing3 = 96,
  devtypMRing2,
  devtypMBar,
  devtypMFrame,
  devtypMWave,
  devtypMRing1 = 127,

  devtypDummy = 255
} devicetype_t;

// Remote type
typedef enum
{
  remotetypUnknown = 224,
  remotetypRFSimply,
  remotetypRFStandard,
  remotetypRFEnhanced,
  remotetypDummy
} remotetype_t;

// Xlight Application Identification
#define XLA_VERSION               0x08
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM
#define XLA_PRODUCT_NAME          "XRemote"                 // Default value. Read from EEPROM


// I_GET_NONCE sub-type
enum {
    SCANNER_PROBE = 0,
    SCANNER_SETUP_RF,           // by NodeID & SubID
    SCANNER_SETUPDEV_RF,        // by UniqueID
    
    SCANNER_GETCONFIG = 8,      // by NodeID & SubID
    SCANNER_SETCONFIG,
    SCANNER_GETDEV_CONFIG,      // by UniqueID
    SCANNER_SETDEV_CONFIG,
    
    SCANNER_TEST_NODE = 16,     // by NodeID & SubID
    SCANNER_TEST_DEVICE,        // by UniqueID
};


typedef struct
{
  UC State                    :8;           // Component state
  UC bmRing                   :8;           // Bitmap for rings, 0 means all rings
  UC BR                       :8;           // Brightness of white [0..100]
  US CCT                      :16;          // CCT (warm or cold) [2700..6500]
  UC R                        :8;           // Brightness of red
  UC G                        :8;           // Brightness of green
  UC B                        :8;           // Brightness of blue
  UC L1                       :8;           // Length of thread 1
  UC L2                       :8;           // Length of thread 2
  UC L3                       :8;           // Length of thread 3
} Hue_t;

typedef struct
{
#if XLA_VERSION <= 0x07
  UC nodeID;                                // Node ID for Remote on specific controller
  UC subID;                                 // SubID
  UC NetworkID[6];
#endif  
  UC deviceID;                              // Device Node ID
  UC subDevID;                              // Device SubID
  UC type;                                  // Type of Device
} DeviceInfo_t;

typedef struct
{
  UC present                  :1;           // 0 - not present; 1 - present
  UC reserved                 :7;
  Hue_t ring;
} DeviceStatus_t;

typedef struct
{
  UC bmDevice                 :4;       // Bitmap of devices, 0 means current device
  UC scenario;                          // ScenarioID or 0 means specific Hue
  UC effect;                            // special effect id, 0 means normal state
  Hue_t hue;
} fnScenario_t;

typedef struct
{
  UC deviceID;                              // Device Node ID
  UC subDevID;                              // Device SubID
  UC keys[NUM_RELAY_KEYS];
  UC state                    :1;           // On / Off
} RelayKeyInfo_t;

#if XLA_VERSION > 0x07
#define XLA_MIN_VER_REQUIREMENT   0x08
typedef struct
{
  // Static & status parameters
  UC version                  :8;           // Data version, other than 0xFF
  UC present                  :1;           // 0 - not present; 1 - present
  UC inPresentation           :1;           // whether in presentation
  UC inConfigMode             :1;           // whether in config mode
  UC reserved0                :5;
  
  // Configurable parameters
  UC nodeID;                                // Node ID for Remote on specific controller
  UC subID;                                 // SubID
  UC NetworkID[6];
  UC rfChannel;                             // RF Channel: [0..127]
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  UC enSDTM                   :1;           // Simple Direct Test Mode Flag
  UC reserved1                :1;
  UC type;                                  // Type of Remote
  US token;                                 // Current token
  UC indDevice                :3;           // Current Device Index: [0..3]
  UC reserved2                :5;
  DeviceInfo_t devItem[NUM_DEVICES];
  fnScenario_t fnScenario[7];
  RelayKeyInfo_t relayKey;
#ifdef HOME_VERSION
  DeviceStatus_t favoritesDevStat[NUM_FAVORITE];
#endif
} Config_t;
#else
#define XLA_MIN_VER_REQUIREMENT   0x03
typedef struct
{
  UC version                  :8;           // Data version, other than 0xFF
  UC indDevice                :3;           // Current Device Index: [0..3]
  UC present                  :1;           // 0 - not present; 1 - present
  UC inPresentation           :1;           // whether in presentation
  UC inConfigMode             :1;           // whether in config mode
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
  UC type;                                  // Type of Remote
  US token;                                 // Current token
  //char Organization[24];                    // Organization name
  //char ProductName[24];                     // Product name
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC enSDTM                   :1;           // Simple Direct Test Mode Flag
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  UC Reserved1                :3;           // Reserved bits
  UC rfChannel;                             // RF Channel: [0..127]
  DeviceInfo_t devItem[NUM_DEVICES];
  fnScenario_t fnScenario[4];
  RelayKeyInfo_t relayKey;
} Config_t;
#endif

extern Config_t gConfig;
extern DeviceStatus_t gDevStatus[NUM_DEVICES];
extern bool gIsChanged;
extern bool gNeedSaveBackup;
extern bool gIsStatusChanged;
extern bool gResetRF;
extern bool gResetNode;

extern uint8_t _uniqueID[UNIQUE_ID_LEN];
extern uint8_t gDelayedOperation;
extern uint8_t gSendScenario;
extern uint8_t gSendDelayTick;

extern int8_t gLastFavoriteIndex;
extern uint8_t gLastFavoriteTick;
#define MAXFAVORITE_INTERVAL 100  // unit is 10ms,1s interval

#define RING_ID_ALL             0
#define RING_ID_1               1
#define RING_ID_2               2
#define RING_ID_3               3

#define IS_SUNNY(DevType)           ((DevType) >= devtypWRing3 && (DevType) <= devtypWRing1)
#define IS_RAINBOW(DevType)         ((DevType) >= devtypCRing3 && (DevType) <= devtypCRing1)
#define IS_MIRAGE(DevType)          ((DevType) >= devtypMRing3 && (DevType) <= devtypMRing1)
#define IS_VALID_REMOTE(DevType)    ((DevType) >= remotetypRFSimply && (DevType) <= remotetypRFEnhanced)

#define IS_GROUP_NODEID(nID)       (nID >= NODEID_MIN_GROUP && nID <= NODEID_MAX_GROUP)
#define IS_NOT_DEVICE_NODEID(nID)  ((nID < NODEID_MIN_DEVCIE || nID > NODEID_MAX_DEVCIE) && nID != NODEID_MAINDEVICE)
#define IS_NOT_REMOTE_NODEID(nID)  (nID < NODEID_MIN_REMOTE || nID > NODEID_MAX_REMOTE)

#if XLA_VERSION > 0x07
#define NodeID(x)                  gConfig.nodeID
#define SubNID(x)                  gConfig.subID
#define NetworkID(x)               gConfig.NetworkID
#else
#define NodeID(x)                  gConfig.devItem[x].nodeID
#define SubNID(x)                  gConfig.devItem[x].subID
#define NetworkID(x)               gConfig.devItem[x].NetworkID
#endif

#define DeviceID(x)                gConfig.devItem[x].deviceID
#define DeviceSubID(x)             gConfig.devItem[x].subDevID
#define DeviceType(x)              gConfig.devItem[x].type
#define DEVST_Present(x)           gDevStatus[x].present
#define DEVST_OnOff(x)             gDevStatus[x].ring.State
#define DEVST_Bright(x)            gDevStatus[x].ring.BR
#define DEVST_WarmCold(x)          gDevStatus[x].ring.CCT
#define DEVST_R(x)                 gDevStatus[x].ring.R
#define DEVST_G(x)                 gDevStatus[x].ring.G
#define DEVST_B(x)                 gDevStatus[x].ring.B

#define CurrentNodeID              NodeID(gConfig.indDevice)
#define CurrentSubNID              SubNID(gConfig.indDevice)
#define CurrentNetworkID           NetworkID(gConfig.indDevice)
#define CurrentDeviceID            DeviceID(gConfig.indDevice)
#define CurrentDevSubID            DeviceSubID(gConfig.indDevice)
#define CurrentDeviceType          DeviceType(gConfig.indDevice)
#define CurrentDevicePresent       DEVST_Present(gConfig.indDevice)
#define CurrentDeviceOnOff         DEVST_OnOff(gConfig.indDevice)
#define CurrentDeviceBright        DEVST_Bright(gConfig.indDevice)
#define CurrentDeviceCCT           DEVST_WarmCold(gConfig.indDevice)
#define CurrentDevice_R            DEVST_R(gConfig.indDevice)
#define CurrentDevice_G            DEVST_G(gConfig.indDevice)
#define CurrentDevice_B            DEVST_B(gConfig.indDevice)

bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen);
bool WaitMutex(uint32_t _timeout);
void RF24L01_IRQ_Handler();
uint8_t ChangeCurrentDevice(uint8_t _newDev);
void UpdateNodeAddress(uint8_t _tx);
bool SendMyMessage();
void EraseCurrentDeviceInfo();
void ToggleSDTM();
void SetConfigMode(bool _sw, uint8_t _devIndex);
bool SayHelloToDevice(bool infinate);

#define IS_MINE_SUBID(nSID)        ((nSID) == 0 || ((nSID) & CurrentDevSubID))

//#define TEST
#ifdef TEST
#define     PC1_Low                GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define     PC3_Low                GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define     PC5_Low                GPIO_ResetBits(GPIOC, GPIO_Pin_5)
#define     PC1_High               GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define     PC3_High               GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define     PC5_High               GPIO_SetBits(GPIOC, GPIO_Pin_5)
#endif

#endif /* __GLOBAL_H */