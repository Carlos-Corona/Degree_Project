#ifndef VARIABLES_H
#define VARIABLES_H

extern double Preheat;
extern int TimePreheat;
extern double Soak;
extern int TimeSoak;
extern double Peak;
extern int TimePeak;
extern double Cooling;
extern int TimeCooling;


enum Command
{
  _None = -1,
  _HandShakeMssage = 0x11, // Handshake Message
  _HandShakeCmmd = 0xCC, // Handshake Command
  _ReadTemp = 0x02, // Set Temperature
  _ReflowStart = 0x03, // Reflow Start
  _KeepWarm = 0x04, // Keep Warm
  _ReflowStatus = 0x05, // Reflow Status
  _ReflowStop = 0x06, // Reflow Stop
  _SetControlSettings = 0x07, // Set Control Settings
  _SetTempSettings = 0x08, // Set Temperature Setting
  _SaveSetiings = 0x0C,
  _UPDATE = 0xFF, // Update Command trigger
};

enum DataType {
  __Preheat = 0x00,
  __Soak = 0x01,
  __Peak = 0x02,
  __KeepWarm = 0x03,
  __Cooling = 0x04,
  __Kp = 0x05,
  __Ki = 0x06,
  __Kd = 0x07,
  __TimePreheat = 0x08,
  __TimeSoak = 0x09,
  __TimePeak = 0x0A,
  __TimeCooling = 0x0B,
  __DataTemp = 0x0C,
};

#endif