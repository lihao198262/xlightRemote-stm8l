#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <stm8l15x.h> //Required for the stdint typedefs
#include "stdio.h"
#include "string.h"
#include "stm8l15x_conf.h"

// Simple Direct Test
// Uncomment this line to work in Simple Direct Test Mode
//#define ENABLE_SDTM

// Config Flashlight and Laser
// Uncomment this line if need Flashlight or Laser Pen
#define ENABLE_FLASHLIGHT_LASER

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
#define NODEID_SMARTPHONE       139
#define NODEID_MIN_GROUP        192
#define NODEID_MAX_GROUP        223
#define NODEID_DUMMY            255
#define BASESERVICE_ADDRESS     0xFE
#define BROADCAST_ADDRESS       0xFF

#define BR_MIN_VALUE            1
#define CT_MIN_VALUE            2700
#define CT_MAX_VALUE            6500
#define CT_SCOPE                38    
#define CT_STEP                 ((CT_MAX_VALUE-CT_MIN_VALUE)/10)

#define UNIQUE_ID_LEN           8
#define NUM_DEVICES             2

#define DELAY_OP_ERASEFLASH     0x10
#define DELAY_OP_PAIRED         0x20
#define DELAY_OP_CONNECTED      0x30
#define DELAY_OP_PPTMODE_ON     0x40
#define DELAY_OP_PPTMODE_OFF    0x50

// Device (lamp) type
typedef enum
{
  devtypUnknown = 0,
  devtypCRing3,     // Color ring - Rainbow
  devtypCRing2,
  devtypCRing1,
  devtypWRing3,     // White ring - Sunny
  devtypWRing2,
  devtypWRing1,
  devtypMRing3 = 8, // Color & Motion ring - Mirage
  devtypMRing2,
  devtypMRing1,
  devtypDummy = 255
} devicetype_t;


// Remote type
typedef enum
{
  remotetypUnknown = 0,
  remotetypRFSimply,
  remotetypRFStandard,
  remotetypRFEnhanced,
  remotetypDummy
} remotetype_t;

typedef struct
{
  UC State                    :1;           // Component state
  UC BR                       :7;           // Brightness of white [0..100]
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
  UC nodeID;                                // Node ID for Remote on specific controller
  UC NetworkID[6];
  UC devcieID;                              // Device Node ID
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
  UC version                  :8;           // Data version, other than 0xFF
  UC indDevice                :3;           // Current Device Index: [0..3]
  UC present                  :1;           // 0 - not present; 1 - present
  UC inPresentation           :1;           // whether in presentation
  UC reserved                 :3;
  UC type;                                  // Type of Remote
  US token;                                 // Current token
  char Organization[24];                    // Organization name
  char ProductName[24];                     // Product name
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC Reserved1                :6;           // Reserved bits
  DeviceInfo_t devItem[NUM_DEVICES];
  UC fnScenario[4];
} Config_t;

extern Config_t gConfig;
extern DeviceStatus_t gDevStatus[NUM_DEVICES];
extern bool gIsChanged;
extern uint8_t _uniqueID[UNIQUE_ID_LEN];
extern uint8_t gDelayedOperation;

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

#define NodeID(x)                  gConfig.devItem[x].nodeID
#define NetworkID(x)               gConfig.devItem[x].NetworkID
#define DeviceID(x)                gConfig.devItem[x].devcieID
#define DeviceType(x)              gConfig.devItem[x].type
#define DEVST_Present(x)           gDevStatus[x].present
#define DEVST_OnOff(x)             gDevStatus[x].ring.State
#define DEVST_Bright(x)            gDevStatus[x].ring.BR
#define DEVST_WarmCold(x)          gDevStatus[x].ring.CCT
#define DEVST_R(x)                 gDevStatus[x].ring.R
#define DEVST_G(x)                 gDevStatus[x].ring.G
#define DEVST_B(x)                 gDevStatus[x].ring.B

#define CurrentNodeID              NodeID(gConfig.indDevice)
#define CurrentNetworkID           NetworkID(gConfig.indDevice)
#define CurrentDeviceID            DeviceID(gConfig.indDevice)
#define CurrentDeviceType          DeviceType(gConfig.indDevice)
#define CurrentDevicePresent       DEVST_Present(gConfig.indDevice)
#define CurrentDeviceOnOff         DEVST_OnOff(gConfig.indDevice)
#define CurrentDeviceBright        DEVST_Bright(gConfig.indDevice)
#define CurrentDeviceCCT           DEVST_WarmCold(gConfig.indDevice)
#define CurrentDevice_R            DEVST_R(gConfig.indDevice)
#define CurrentDevice_G            DEVST_G(gConfig.indDevice)
#define CurrentDevice_B            DEVST_B(gConfig.indDevice)

bool WaitMutex(uint32_t _timeout);
void UpdateNodeAddress(void);
void RF24L01_IRQ_Handler();
uint8_t ChangeCurrentDevice(uint8_t _newDev);
void UpdateNodeAddress();
bool SendMyMessage();
void EraseCurrentDeviceInfo();
bool SayHelloToDevice(bool infinate);

#endif /* __GLOBAL_H */