#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"
#include "button.h"

uint8_t bMsgReady = 0;

void MsgScanner_ProbeAck();
void MsgScanner_ConfigAck(uint8_t offset,uint8_t cfglen,bool _isByUniqueid);
void Process_SetConfig(u8 _len);
void Process_SetDevConfig(u8 _len);
void Process_SetupRF(const UC *rfData,uint8_t rflen);

bool SendCfgBlock(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid);
typedef struct
{
  uint8_t offset;
  uint8_t size;
  uint8_t byUniqueid;  // whether getconfig by uniqueid
}CfgBlock;
#define OUT_CFG_MESSAGE_LEN           16
CfgBlock out_cfg_msg_buf[OUT_CFG_MESSAGE_LEN];
u8 cfg_msg_out_buf_read_ptr = 0;
u8 cfg_msg_out_buf_write_ptr = 0;

bool AddCfgOutputBuf(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid) {  
  CfgBlock cfgblock;
  cfgblock.offset = offset;
  cfgblock.size = size;
  cfgblock.byUniqueid = isNeedUniqueid;
  out_cfg_msg_buf[cfg_msg_out_buf_write_ptr++] = cfgblock;
  cfg_msg_out_buf_write_ptr %= OUT_CFG_MESSAGE_LEN;
  return TRUE;
}

bool ProcessOutputCfgMsg() {
  // Send output Cfg msg
  while( cfg_msg_out_buf_read_ptr != cfg_msg_out_buf_write_ptr) {   
    CfgBlock cfgblock = out_cfg_msg_buf[cfg_msg_out_buf_read_ptr++];
    SendCfgBlock(cfgblock.offset,cfgblock.size,TRUE);
    cfg_msg_out_buf_read_ptr %= OUT_CFG_MESSAGE_LEN;
  }
  return TRUE;
}

bool SendCfgBlock(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid) {
  // Send output Cfg msg  
    build(NODEID_RF_SCANNER, gConfig.subID, C_INTERNAL, I_GET_NONCE_RESPONSE, 0, 1);
    // Common payload
    sndMsg.payload.data[0] = SCANNER_GETDEV_CONFIG;
    sndMsg.payload.data[1] = offset;
    uint8_t custom_playload = 2;
    if(isNeedUniqueid != 0) 
    {
      memcpy(sndMsg.payload.data + 2,_uniqueID, UNIQUE_ID_LEN);
      custom_playload += UNIQUE_ID_LEN;
    }  
    memcpy(sndMsg.payload.data + custom_playload, (void *)((uint16_t)(&gConfig) + offset), size);
    moSetLength(size+custom_playload);
    moSetPayloadType(P_CUSTOM);
    bMsgReady = 1;
    SendMyMessage();
}

// Assemble message
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck)
{
    sndMsg.header.version_length = PROTOCOL_VERSION;
    sndMsg.header.sender = CurrentNodeID;
    sndMsg.header.destination = _destination;
    sndMsg.header.sensor = _sensor;
    sndMsg.header.type = _type;
    moSetCommand(_command);
    moSetRequestAck(_enableAck);
    moSetAck(_isAck);
}

void UpdateCurrentDeviceOnOff(bool _OnOff) {
  CurrentDeviceOnOff = _OnOff;
  /*
  if( _OnOff ) {
    // Automatically clear the flag, so that the remote resumes controlling the lights
    gConfig.inPresentation = 0;
  }*/
}

void UpdateNodeID(uint8_t nID) {
  CurrentNodeID = nID;
  if( gConfig.indDevice == 0 ) {
    for( uint8_t i = 1; i < NUM_DEVICES; i++ ) {
      if( DeviceID(i) == 0 )
        DeviceID(i) = CurrentNodeID;
    }
  }
}

void UpdateSubID(uint8_t nID) {
  CurrentSubNID = nID;
  if( gConfig.indDevice == 0 ) {
    for( uint8_t i = 1; i < NUM_DEVICES; i++ ) {
      if( DeviceSubID(i) == 0 )
        DeviceSubID(i) = CurrentDevSubID;
    }
  }
}

void SaveFavoriteDevStatus()
{
#ifdef NUM_FAVORITE
  if(gLastFavoriteIndex >= NUM_FAVORITE) return;
  if(gLastFavoriteIndex == -1) return;
    gConfig.favoritesDevStat[gLastFavoriteIndex].reserved = 1;
    gConfig.favoritesDevStat[gLastFavoriteIndex].ring.BR = CurrentDeviceBright;
    gConfig.favoritesDevStat[gLastFavoriteIndex].ring.R = CurrentDevice_R;
    gConfig.favoritesDevStat[gLastFavoriteIndex].ring.G = CurrentDevice_G;
    gConfig.favoritesDevStat[gLastFavoriteIndex].ring.B = CurrentDevice_B;
    gConfig.favoritesDevStat[gLastFavoriteIndex].ring.CCT = CurrentDeviceCCT;
    gLastFavoriteTick = MAXFAVORITE_INTERVAL;
    gIsChanged = TRUE;
#endif
}

uint8_t ParseProtocol(){
  if( rcvMsg.header.destination != CurrentNodeID && rcvMsg.header.destination != BROADCAST_ADDRESS ) return 0;
  
  uint8_t _cmd = miGetCommand();
  uint8_t _sender = rcvMsg.header.sender;  // The original sender
  uint8_t _type = rcvMsg.header.type;
  uint8_t _sensor = rcvMsg.header.sensor;
  uint8_t _lenPayl = miGetLength();
  bool _isAck = (bool)miGetAck();
  
  switch( _cmd ) {
  case C_INTERNAL:
    if( _type == I_ID_RESPONSE ) {
      // Device/client got nodeID from Controller
      uint8_t lv_nodeID = _sensor;
      if( IS_NOT_REMOTE_NODEID(lv_nodeID) && !IS_GROUP_NODEID(lv_nodeID) ) {
      } else {
        if( _lenPayl > 8 ) {
          // Verify _uniqueID        
          if(!isIdentityEqual(_uniqueID, rcvMsg.payload.data+8, UNIQUE_ID_LEN)) {
            return 0;
          }
        }
        UpdateNodeID(lv_nodeID);
        memcpy(CurrentNetworkID, rcvMsg.payload.data, sizeof(CurrentNetworkID));
        UpdateNodeAddress(NODEID_GATEWAY);
        gIsChanged = TRUE;
        gDelayedOperation = DELAY_OP_PAIRED;
        Msg_Presentation();
        return 1;
      }
    } else if( _type == I_CONFIG ) {
      // Node Config
      switch( _sensor ) {
      case NCF_QUERY:
        // Inform controller with version & NCF data
        Msg_NodeConfigData(_sender);
        return 1;
        break;

      case NCF_DEV_ASSOCIATE:
        CurrentDeviceID = rcvMsg.payload.data[0];
        CurrentDevSubID = rcvMsg.payload.data[1];
        break;

      case NCF_DEV_SET_SUBID:
        UpdateSubID(rcvMsg.payload.data[0]);        
        break;

      case NCF_DEV_SET_RELAY_NODE:
        gConfig.relayKey.deviceID = rcvMsg.payload.data[0];
        gConfig.relayKey.subDevID = rcvMsg.payload.data[1];
        break;

      case NCF_DEV_SET_RELAY_KEYS:
        {
          uint8_t _len = (_lenPayl > NUM_RELAY_KEYS ? NUM_RELAY_KEYS : _lenPayl);
          memset(gConfig.relayKey.keys, 0x00, NUM_RELAY_KEYS);
          for( uint8_t i = 0; i < _len; i++ ) {
            gConfig.relayKey.keys[i] = rcvMsg.payload.data[i];
          }
        }
        break;
        
      case NCF_DEV_CONFIG_MODE:
        SetConfigMode(rcvMsg.payload.data[0], rcvMsg.payload.data[1]);
        break;
        
      case NCF_DEV_MAX_NMRT:
        gConfig.rptTimes = rcvMsg.payload.data[0];
        break;
        
      case NCF_DATA_FN_SCENARIO:
        {
          uint8_t fn_id = rcvMsg.payload.data[0] & 0x0F;
          if( fn_id < 4 ) {
            gConfig.fnScenario[fn_id].bmDevice = (rcvMsg.payload.data[0] >> 4);
            gConfig.fnScenario[fn_id].scenario = rcvMsg.payload.data[1];
          } else {
            return 0;
          }
        }
        break;
      case NCF_DATA_FN_HUE:
        {
          uint8_t fn_id = rcvMsg.payload.data[0] & 0x0F;
          if( fn_id < 4 ) {
            gConfig.fnScenario[fn_id].bmDevice = (rcvMsg.payload.data[0] >> 4);
            gConfig.fnScenario[fn_id].scenario = 0;
            memcpy(&(gConfig.fnScenario[fn_id].hue), rcvMsg.payload.data+1, sizeof(Hue_t));
          } else {
            return 0;
          }
        }
        break;
      }
      gIsChanged = TRUE;
      Msg_NodeConfigAck(_sender, _sensor);
      return 1;
    }else if( _type == I_GET_NONCE ) {
      // RF Scanner Probe
        if( _sender == NODEID_RF_SCANNER ) {
          uint8_t _lenPayl = miGetLength();
          if( rcvMsg.payload.data[0] == SCANNER_PROBE ) {      
            MsgScanner_ProbeAck();
          } else if( rcvMsg.payload.data[0] == SCANNER_SETUP_RF ) {
          if(!IS_MINE_SUBID(_sensor)) return 0;  
            Process_SetupRF(rcvMsg.payload.data + 1,_lenPayl-1);
          }else if( rcvMsg.payload.data[0] == SCANNER_SETUPDEV_RF ) {
            if(!isIdentityEqual(rcvMsg.payload.data + 1,_uniqueID,UNIQUE_ID_LEN)) return 0;
            Process_SetupRF(rcvMsg.payload.data + 1 + UNIQUE_ID_LEN,_lenPayl-1 - UNIQUE_ID_LEN);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_SETCONFIG ) {
            
            if(!IS_MINE_SUBID(_sensor)) return 0;          
            uint8_t cfg_len = _lenPayl - 2;
            Process_SetConfig(cfg_len);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_SETDEV_CONFIG ) {  
            if(!isIdentityEqual(rcvMsg.payload.data + 2,_uniqueID,UNIQUE_ID_LEN)) return 0;
            uint8_t cfg_len = _lenPayl - 10;
            Process_SetDevConfig(cfg_len);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_GETDEV_CONFIG ) {  
            uint8_t offset = rcvMsg.payload.data[1];
            uint8_t cfgblock_len = rcvMsg.payload.data[10];
            if(!isIdentityEqual(rcvMsg.payload.data + 2,_uniqueID,UNIQUE_ID_LEN)) return 0;
            MsgScanner_ConfigAck(offset,cfgblock_len,TRUE); 
          }
          else if( rcvMsg.payload.data[0] == SCANNER_GETCONFIG ) { 
            if(!IS_MINE_SUBID(_sensor)) return 0;  
            uint8_t offset = rcvMsg.payload.data[1];
            uint8_t cfgblock_len = rcvMsg.payload.data[2];
            MsgScanner_ConfigAck(offset,cfgblock_len,FALSE);
          }
          return 1;
        }      
      }
    break;
      
  case C_PRESENTATION:
    if( _sensor == S_DIMMER ) {
      if( _isAck ) {
        // Device/client got Response to Presentation message, ready to work
        gConfig.token = rcvMsg.payload.uiValue;
        gConfig.present = (gConfig.token >  0);
        if(!IS_NOT_DEVICE_NODEID(_type) || IS_GROUP_NODEID(_type) ) {
          CurrentDeviceID = _type;
        }
        gIsStatusChanged = TRUE;
        if( gDelayedOperation != DELAY_OP_PAIRED ) gDelayedOperation = DELAY_OP_CONNECTED;
        
        // REQ device status
        Msg_RequestDeviceStatus();
        return 1;
      }
    }
    break;
  
  case C_REQ:
  case C_SET:
    if( _isAck ) {
      if( _type == V_STATUS ) {
        bool _OnOff = rcvMsg.payload.bValue;
        if( _OnOff != CurrentDeviceOnOff ) {
          UpdateCurrentDeviceOnOff(_OnOff);
          gIsStatusChanged = TRUE;
          // ToDo: change On/Off LED
        }
      } else if( _type == V_PERCENTAGE ) {
        if( rcvMsg.payload.data[1] != CurrentDeviceBright || rcvMsg.payload.data[0] != CurrentDeviceOnOff) {
          CurrentDeviceBright = rcvMsg.payload.data[1];
          UpdateCurrentDeviceOnOff(rcvMsg.payload.data[0]);
          gIsStatusChanged = TRUE;
          // ToDo: change On/Off LED
        }        
      } else if( _type == V_LEVEL ) { // CCT
        uint16_t _CCTValue = rcvMsg.payload.data[1] * 256 + rcvMsg.payload.data[0];
        if( _CCTValue != CurrentDeviceCCT ) {
          CurrentDeviceCCT = _CCTValue;
          gIsStatusChanged = TRUE;
        }
      } else if( _type == V_RGBW ) {
        if( rcvMsg.payload.data[0] ) { // Success
          CurrentDeviceType = rcvMsg.payload.data[1];
          CurrentDevicePresent = rcvMsg.payload.data[2];
          // uint8_t _RingID = rcvMsg.payload.data[3];     // No use for now
          CurrentDeviceOnOff = rcvMsg.payload.data[4];
          CurrentDeviceBright = rcvMsg.payload.data[5];
          if( IS_SUNNY(CurrentDeviceType) ) {
            uint16_t _CCTValue = rcvMsg.payload.data[7] * 256 + rcvMsg.payload.data[6];
            CurrentDeviceCCT = _CCTValue;
          } else if( IS_RAINBOW(CurrentDeviceType) || IS_MIRAGE(CurrentDeviceType) ) {
            // Set RGBW
            CurrentDeviceCCT = rcvMsg.payload.data[6];
            CurrentDevice_R = rcvMsg.payload.data[7];
            CurrentDevice_G = rcvMsg.payload.data[8];
            CurrentDevice_B = rcvMsg.payload.data[9];
          }
          gIsStatusChanged = TRUE;
          // ToDo: change On/Off LED
          // save favorite device status in MAXFAVORITE_INTERVAL
          if(gLastFavoriteTick < MAXFAVORITE_INTERVAL)
          {
            SaveFavoriteDevStatus();
          }       
        }
      }
    }    
    break;
  }
  
  return 0;
}

void Msg_NodeConfigAck(uint8_t _to, uint8_t _ncf) {
  build(_to, _ncf, C_INTERNAL, I_CONFIG, 0, 1);

  sndMsg.payload.data[0] = 1;      // OK
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  bMsgReady = 1;
}

// Prepare NCF query ack message
void Msg_NodeConfigData(uint8_t _to) {
  uint8_t payl_len = 0;
  build(_to, NCF_QUERY, C_INTERNAL, I_CONFIG, 0, 1);

  sndMsg.payload.data[payl_len++] = gConfig.version;
  sndMsg.payload.data[payl_len++] = gConfig.type;
  sndMsg.payload.data[payl_len++] = ((gConfig.indDevice << 5) | (gConfig.inPresentation << 4) | gConfig.rptTimes);
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  for(uint8_t _index = 0; _index < NUM_DEVICES; _index++ ) {
    sndMsg.payload.data[payl_len++] = DeviceID(_index);
  }
  sndMsg.payload.data[payl_len++] = gConfig.fnScenario[0].scenario;
  sndMsg.payload.data[payl_len++] = gConfig.fnScenario[1].scenario;
  sndMsg.payload.data[payl_len++] = gConfig.fnScenario[2].scenario;
  sndMsg.payload.data[payl_len++] = gConfig.fnScenario[3].scenario;
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}

void Msg_RequestNodeID() {
  // Request NodeID for remote
  build(BASESERVICE_ADDRESS, NODE_TYP_REMOTE, C_INTERNAL, I_ID_REQUEST, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation() {
  build(NODEID_GATEWAY, S_DIMMER, C_PRESENTATION, gConfig.type, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Enquiry Device Status
void Msg_RequestDeviceStatus() {
  build(CurrentDeviceID, CurrentDevSubID, C_REQ, V_RGBW, 1, 0);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = RING_ID_ALL;
  bMsgReady = 1;
}

// Set current device 1:On; 0:Off; 2:toggle
void Msg_DevOnOff(uint8_t _sw) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_STATUS, 1, 0);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = _sw;
  bMsgReady = 1;
}

void Msg_DevOnOffDelay(uint8_t _sw,uint8_t _unit,uint8_t time)
{
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_STATUS, 1, 0);
  moSetLength(3);
  moSetPayloadType(P_CUSTOM);
  sndMsg.payload.data[0] = _sw;      // State: On
  sndMsg.payload.data[1] = _unit;                
  sndMsg.payload.data[2] = time;
  bMsgReady = 1;
}

// Set device 1:On; 0:Off; 2:toggle
void Msg_SpecialDevOnOff(uint8_t nodeid,uint8_t subid,uint8_t _sw) {
  SendMyMessage();
  build(nodeid, subid, C_SET, V_STATUS, 1, 0);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = _sw;
  bMsgReady = 1;
}

// Set relay key (e.g. pot light) 1:On; 0:Off; 2:toggle
void Msg_RelayOnOff(uint8_t _sw) {
  if( gConfig.relayKey.deviceID > 0 ) {
    SendMyMessage();
    if( _sw == DEVICE_SW_TOGGLE ) {
      gConfig.relayKey.state = 1 - gConfig.relayKey.state;
    } else {
      gConfig.relayKey.state = (_sw == DEVICE_SW_ON);
    }
    uint8_t _len = 0;
    build(gConfig.relayKey.deviceID, gConfig.relayKey.subDevID, C_SET, (gConfig.relayKey.state ? V_RELAY_ON : V_RELAY_OFF), 0, 0);
    moSetPayloadType(P_BYTE);
    for( _len = 0; _len < NUM_RELAY_KEYS; _len++ ) {
      if( gConfig.relayKey.keys[_len] == 0 ) break;
      sndMsg.payload.data[_len] = gConfig.relayKey.keys[_len];
    }
    moSetLength(_len);
    bMsgReady = 1;
  }
}

// Set current device brightness
void Msg_DevBrightness(uint8_t _op, uint8_t _br) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_PERCENTAGE, 1, 0);
  moSetLength(2);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.data[0] = _op;
  sndMsg.payload.data[1] = _br;
  bMsgReady = 1;
}

// Set current device CCT
void Msg_DevCCT(uint8_t _op, uint16_t _cct) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_LEVEL, 1, 0);
  moSetLength(3);
  moSetPayloadType(P_UINT16);
  sndMsg.payload.data[0] = _op;
  sndMsg.payload.data[1] = _cct % 256;
  sndMsg.payload.data[2] = _cct / 256;
  bMsgReady = 1;
}

// Set current device brightness & CCT
void Msg_DevBR_CCT(uint8_t _br, uint16_t _cct) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_RGBW, 1, 0);
  moSetLength(5);
  moSetPayloadType(P_CUSTOM);
  sndMsg.payload.data[0] = RING_ID_ALL;      // Ring ID: 0 means all rings
  sndMsg.payload.data[1] = 1;                // State: On
  sndMsg.payload.data[2] = _br;
  sndMsg.payload.data[3] = _cct % 256;
  sndMsg.payload.data[4] = _cct / 256;
  bMsgReady = 1;
}

// Set current device brightness & RGBW
void Msg_DevBR_RGBW(uint8_t _br, uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _w) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_RGBW, 1, 0);
  moSetLength(7);
  moSetPayloadType(P_CUSTOM);
  sndMsg.payload.data[0] = RING_ID_ALL;      // Ring ID: 0 means all rings
  sndMsg.payload.data[1] = 1;                // State: On
  sndMsg.payload.data[2] = _br;
  sndMsg.payload.data[3] = _w;
  sndMsg.payload.data[4] = _r;
  sndMsg.payload.data[5] = _g;
  sndMsg.payload.data[6] = _b;
  bMsgReady = 1;
}

// Change scenario on current device
void Msg_DevScenario(uint8_t _scenario) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_SCENE_ON, 1, 0);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = _scenario;
  bMsgReady = 1;
}

// Change special effect on current device
void Msg_DevSpecialEffect(uint8_t _effect) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_VAR1, 1, 0);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = _effect;
  bMsgReady = 1;
}

// PPT Object Action
void Msg_PPT_ObjAction(uint8_t _obj, uint8_t _action) {
  SendMyMessage();
  build(CurrentDeviceID, NODEID_PROJECTOR, C_SET, V_STATUS, 1, 0);
  moSetLength(4);
  moSetPayloadType(P_STRING);
  sndMsg.payload.data[0] = _obj;
  sndMsg.payload.data[1] = _action;
  sndMsg.payload.data[2] = '0';      // Reserved
  sndMsg.payload.data[3] = '0';      // Reserved
  bMsgReady = 1;
}

//----------------------------------------------
// RF Scanner Messages
//----------------------------------------------
// Probe ack message
void MsgScanner_ProbeAck() {
  uint8_t payl_len = UNIQUE_ID_LEN + 1;
  build(NODEID_RF_SCANNER, 0x00, C_INTERNAL, I_GET_NONCE_RESPONSE, 0, 1);

  // Common payload
  sndMsg.payload.data[0] = SCANNER_PROBE;
  memcpy(sndMsg.payload.data + 1, _uniqueID, UNIQUE_ID_LEN);
  
  sndMsg.payload.data[payl_len++] = gConfig.version;
  sndMsg.payload.data[payl_len++] = gConfig.type;
  sndMsg.payload.data[payl_len++] = gConfig.nodeID;
  sndMsg.payload.data[payl_len++] = gConfig.subID;
  sndMsg.payload.data[payl_len++] = gConfig.rfChannel;
  sndMsg.payload.data[payl_len++] = (gConfig.rfDataRate << 2) + gConfig.rfPowerLevel;
  memcpy(sndMsg.payload.data + payl_len, gConfig.NetworkID, sizeof(gConfig.NetworkID));
  payl_len += sizeof(gConfig.NetworkID);
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;
//    uint8_t uniqueid[8];
//    UC ConfigBlock[15];
//}MyMsgPayload_t
#define CFGBLOCK_SIZE    15
#define CFGBLOCK_NO_UNIQUEID_SIZE CFGBLOCK_SIZE+UNIQUE_ID_LEN
void MsgScanner_ConfigAck(uint8_t offset,uint8_t cfglen,bool _isByUniqueid) {
  uint8_t cfg_end_offset = cfglen;
  if(cfglen == 0) cfg_end_offset = sizeof(Config_t)-1;
  else
  {
    cfg_end_offset = offset + cfglen > sizeof(Config_t)-1?sizeof(Config_t)-1:offset + cfglen;
  }  
  while( offset < cfg_end_offset )
  {
    uint8_t left_len = cfg_end_offset - offset;
    uint8_t payl_len = left_len < CFGBLOCK_SIZE ? left_len : CFGBLOCK_SIZE;
    if(_isByUniqueid) AddCfgOutputBuf(offset,payl_len,1);
    else  
    {
      payl_len = left_len < CFGBLOCK_NO_UNIQUEID_SIZE ? left_len : CFGBLOCK_NO_UNIQUEID_SIZE;
      AddCfgOutputBuf(offset,payl_len,0);
    }
    offset+=payl_len;
    offset %= sizeof(Config_t);
  }
}

//////set config by nodeid&subid data struct/////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;  //config offset
//    UC ConfigBlock[23];
//}MyMsgPayload_t
//////set config by nodeid&subid data struct/////////////////////
void Process_SetConfig(u8 _len) {
  uint8_t offset = rcvMsg.payload.data[1];
  memcpy((void *)((uint16_t)(&gConfig) + offset),rcvMsg.payload.data+2,_len);
  gIsChanged = TRUE;
}
//////set config by uniqueid data struct/////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;   //config offset
//    uint8_t uniqueid[8];
//    
//    UC ConfigBlock[15];
//}MyMsgPayload_t
//////set config by uniqueid data struct/////////////////////
void Process_SetDevConfig(u8 _len) {
    uint8_t offset = rcvMsg.payload.data[1];
    memcpy((void *)((uint16_t)(&gConfig) + offset),rcvMsg.payload.data+2+UNIQUE_ID_LEN,_len);
    gIsChanged = TRUE;
}
bool IsNodeidValid(uint8_t nodeid)
{
  return !(IS_NOT_REMOTE_NODEID(nodeid));
}
//////set rf /////////////////////////////////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t uniqueid[8];
//    uint8_t channel;
//    uint8_t datarate;
//    uint8_t powerlevel;
//    uint8_t network[6];
//    uint8_t nodeid;        //unnecessary data field£¬has this field£¬need change nodeid£¬0 indicate ignore this parameter
//    uint8_t subid;         //unnecessary data field£¬has this field£¬need change subid
//}MyMsgPayload_t
//////set rf /////////////////////////////////////////////////
void Process_SetupRF(const UC *rfData,uint8_t rflen)
{
  bool bNeedChangeCfg = FALSE;
  if(rflen > 0 &&(*rfData)>=0 && (*rfData)<=127)
  {
    if(gConfig.rfChannel != (*rfData))
    {
      gConfig.rfChannel = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  if(rflen > 1 &&(*rfData)>=RF24_1MBPS && (*rfData)<= RF24_250KBPS)
  {
    if(gConfig.rfDataRate != (*rfData))
    {
      gConfig.rfDataRate = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  if(rflen > 2 &&(*rfData)>=RF24_PA_MIN && (*rfData)<= RF24_PA_ERROR)
  {
    if(gConfig.rfPowerLevel != (*rfData))
    {
      gConfig.rfPowerLevel = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  bool bValidNet = FALSE;
  bool newNetwork[6] = {0};
  if(rflen > 8)
  {  
    for(uint8_t i = 0;i<6;i++)
    {
      if(*(rfData+i) != 0)
      {
        bValidNet=TRUE;
        break;
      }
    }
    if(isIdentityEqual(rfData,gConfig.NetworkID,sizeof(gConfig.NetworkID)))
    {
      bValidNet=FALSE;
    }
    else
    {
      memcpy(newNetwork,rfData,sizeof(newNetwork));
    }
  }
  rfData = rfData + sizeof(gConfig.NetworkID);
  bool bNeedResetNode = FALSE;
  if(rflen > 9 && (* rfData) != 0)
  {
    if(gConfig.nodeID != (* rfData))
    {
      if(IsNodeidValid(*rfData))
      {
        gConfig.nodeID = (* rfData);
        bNeedResetNode = TRUE;
      }    
    }
  }
  rfData++; 
  if(rflen > 10)
  {
    if(gConfig.subID != (* rfData ))
    {
      gConfig.subID = (*rfData);
      bNeedChangeCfg = TRUE;
    }
  }
  if(bValidNet)
  {// nodeid is valid,allow change networkid
    if(IsNodeidValid(gConfig.nodeID))
    {
      memcpy(gConfig.NetworkID,newNetwork,sizeof(gConfig.NetworkID));
      bNeedResetNode = TRUE;
    }
  }
  if(bNeedResetNode)
    gResetNode = TRUE;
  if(gResetNode || gResetRF || bNeedChangeCfg)
  {
    gIsChanged = TRUE;
  }
}
//----------------------------------------------
