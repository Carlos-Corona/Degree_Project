#include "I2C.h"

void pinRequestInterrupt()
{
  digitalWrite(INTERRUPT_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(INTERRUPT_PIN, LOW);
}

void receiveEvent(int _numBytes)
{
  if (Wire.available() && _numBytes == sizeof(RxData))
  {
    Wire.readBytes((byte *)&RxData, sizeof(RxData));
  }
  Serial.print("command: ");
  Serial.println(RxData.command);
  Serial.print(" - data: ");
  for (int i = 0; i < 13; i++)
  {
    Serial.print(RxData.data[i]);
    Serial.print(" ");
  }
}

void sendEvent(double data[13], int _command_, I2CData &Txdata)
{
  Txdata.command = _command_;
  for (int i = 0; i < 13; i++)
  {
    Txdata.data[i] = data[i];
  }
  pinRequestInterrupt();
}

void requestEvent()
{
  Wire.write((byte *)&TxData, sizeof(TxData));
}

void HandShake()
{
  _command_ = _HandShakeCmmd;
  data[DataTemp] = _HandShakeMssage;
  sendEvent(data, _command_, TxData);
}