#ifndef I2C_H
#define I2C_H
#include <Arduino.h>
#include <Wire.h>
#include <../Device_Setup.h>

enum Command
{
  _HandShakeMssage = 0x00, // Handshake Message
  _HandShakeCmmd = 0x01, // Handshake Command
  _ReadTemp = 0x02, // Set Temperature
  _ReflowStart = 0x03, // Reflow Start
  _KeepWarm = 0x04, // Keep Warm
  _ReflowStatus = 0x05, // Reflow Status
  _ReflowStop = 0x06, // Reflow Stop
  _SetControlSettings = 0x07, // Set Control Settings
  _SetTempSettings = 0x08, // Set Temperature Setting
  _UPDATE = 0xFF, // Update Command trigger
};

enum DataType {
  Preheat = 0x00,
  Soak = 0x01,
  Peak = 0x02,
  Cooling = 0x03,
  KeepWarm = 0x04,
  TimePreheat = 0x05,
  TimeSoak = 0x06,
  TimePeak = 0x07,
  TimeCooling = 0x08,
  Kp = 0x09,
  Ki = 0x0A,
  Kd = 0x0B,
  DataTemp = 0x0C,
};

struct I2CData
{
  int command;
  double data[13] = {0};
};

int _action;
int _command_;
double data[13] = {0};
I2CData TxData;
I2CData RxData;


void pinRequestInterrupt();
void receiveEvent(int);
void sendEvent(double data[13], int _command_, I2CData &Txdata, DataType dataType);

void requestEvent();
void HandShake();
#endif