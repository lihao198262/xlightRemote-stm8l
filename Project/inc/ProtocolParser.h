#ifndef __PROTOCOL_PARSER_H
#define __PROTOCOL_PARSER_H

#include "_global.h"

extern uint8_t bMsgReady;

uint8_t ParseProtocol();
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck);
void Msg_NodeConfigAck(uint8_t _to, uint8_t _ncf);
void Msg_NodeConfigData(uint8_t _to);
void Msg_RequestNodeID();
void Msg_Presentation();
void Msg_RequestDeviceStatus();
void Msg_DevOnOff(uint8_t _sw);
void Msg_RelayOnOff(uint8_t _sw);
void Msg_DevBrightness(uint8_t _op, uint8_t _br);
void Msg_DevCCT(uint8_t _op, uint16_t _cct);
void Msg_DevBR_CCT(uint8_t _br, uint16_t _cct);
void Msg_DevBR_RGBW(uint8_t _br, uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _w);
void Msg_DevScenario(uint8_t _scenario);
void Msg_PPT_ObjAction(uint8_t _obj, uint8_t _action);

bool ProcessOutputCfgMsg();

#endif /* __PROTOCOL_PARSER_H */