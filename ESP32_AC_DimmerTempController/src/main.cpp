#include <Arduino.h>
#include <Wire.h>
#include <MAX6675.h>
#include <RBDdimmer.h>
#include <SPI.h>
#include <PID.h>
#include <../variables.h>
#include <../Device_Setup.h>

byte address_Slave = I2C_SlaveAddress;

///////////////// I2C ///////////////////////

struct I2CData
{
  int command;
  double data[13] = {0};
};

int _command_ = _None;
double local_data[13] = {0};
I2CData TxData;
I2CData RxData;

void pinRequestInterrupt();
void receiveEvent(int);
void sendEvent(double data[13], int _command_, I2CData &Txdata);

void requestEvent();
void HandShake();

///////////////////////////////////////////

double linear_interp(double x, double x0, double x1, double y0, double y1);
void calculate_reflow_profile(double temp_reflow_individual[360], double local_data[13]);
double temp_reflow[360] = {0};
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

float readTemperature();

float Kp = 20;
float Ki = 0.2;
float Kd = 200;
float prev_Error = 0;
float T = 1;
float referenceTemp = 0;
float current_temperature = 0;
int step = 0;
PIDController pid(Kp, Ki, Kd, prev_Error, T);

float control_ACDimer(double lecture_sensor, double ReferenceTemperature);
void ReflowProcess();
void KeepProcess();
dimmerLamp dimmerL1(CHANNEL_L1, ZEROCROSS);
dimmerLamp dimmerL2(CHANNEL_L2, ZEROCROSS);
void setup()
{
  Serial.begin(115200);
  dimmerL1.begin(NORMAL_MODE, ON);
  dimmerL2.begin(NORMAL_MODE, ON);
  pinMode(INTERRUPT_PIN, OUTPUT);
  digitalWrite(INTERRUPT_PIN, LOW);
  while (!Wire.begin(I2C_SlaveAddress, SDA_PIN, SCL_PIN, FastMode))
  {
    true;
    delay(100);
    Serial.println("I2C Communication Waiting...");
  }
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  delay(1000);
  dimmerL1.setPower(0);
  dimmerL2.setPower(0);
  dimmerL1.setState(OFF);
  dimmerL2.setState(OFF);
  Serial.println("Serial Communication Ready...");
  Serial.println("ESP32 I2C Slave");
}

void loop()
{

  switch (_command_)
  {
  case _None:
    delay(100);
    break;
  case _ReflowStart:
    Serial.println("Reflow Start");
    dimmerL1.setState(ON);
    dimmerL2.setState(ON);
    calculate_reflow_profile(temp_reflow, local_data);
    ReflowProcess();
    break;
  case _KeepWarm:
    Serial.println("Keep Warm");
    dimmerL1.setState(ON);
    dimmerL2.setState(ON);
    calculate_reflow_profile(temp_reflow, local_data);
    KeepProcess();
    break;
  case _ReflowStop:
    Serial.println("Reflow Stop");
    dimmerL1.setPower(0);
    dimmerL2.setPower(0);
    dimmerL1.setState(OFF);
    dimmerL2.setState(OFF);
    local_data[__DataTemp] = readTemperature();
    sendEvent(local_data, _ReadTemp, TxData);
    delay(999);
    break;
  case _SaveSetiings:
    Serial.println("SaveSettings");
    pid.SetPIDTunning(local_data[__Kp], local_data[__Ki], local_data[__Kd]);
    _command_ = _None;
    break;
  case _HandShakeCmmd:
    Serial.println("HandShake");
    HandShake();
    _command_ = _None;
    break;
  default:
    _command_ = _None;
    break;
  }
  delay(5);
}

void ReflowProcess()
{
  int i = 0;
  static unsigned long lastTempSampleTime = 0;
  static unsigned long lastPIDUpdateTime = 0;
  int num_sample = 0;
  while (_command_ == _ReflowStart)
  {
    unsigned long currentTime = millis();
    if (currentTime - lastTempSampleTime >= 250)
    {
      num_sample += 1;
      current_temperature = (current_temperature + readTemperature());
      lastTempSampleTime = currentTime;
    }

    if (currentTime - lastPIDUpdateTime >= 1000)
    {
      current_temperature = current_temperature / num_sample;
      float duty_scale = control_ACDimer(current_temperature, temp_reflow[i]);
      current_temperature = round(current_temperature);
      Serial.print("Duty scale: ");
      Serial.println(duty_scale);
      Serial.print("Reference scale: ");
      Serial.println(temp_reflow[i]);
      dimmerL1.setPower(duty_scale);
      dimmerL2.setPower(duty_scale);
      if (duty_scale < 5)
      {
        dimmerL1.setState(OFF);
        dimmerL2.setState(OFF);
      }else{
        dimmerL1.setState(ON);
        dimmerL2.setState(ON);
      }
      local_data[__DataTemp] = current_temperature;
      sendEvent(local_data, _ReadTemp, TxData);
      current_temperature = 0;
      num_sample = 0;
      i += 1;
      lastPIDUpdateTime = currentTime;
    }
    if (i > 359)
    {
      sendEvent(local_data, _ReflowStop, TxData);
      i = 359;
    }
  }
}

void KeepProcess()
{
  int i = 0;
  static unsigned long lastTempSampleTime = 0;
  static unsigned long lastPIDUpdateTime = 0;
  int num_sample = 0;
  while (_command_ == _KeepWarm)
  {
    unsigned long currentTime = millis();
    if (currentTime - lastTempSampleTime >= 250)
    {
      num_sample += 1;
      current_temperature = (current_temperature + readTemperature());
      lastTempSampleTime = currentTime;
    }

    if (currentTime - lastPIDUpdateTime >= 1000)
    {
      current_temperature = current_temperature / num_sample;
      float duty_scale = control_ACDimer(current_temperature, local_data[__KeepWarm]);
      current_temperature = round(current_temperature);
      dimmerL1.setPower(duty_scale);
      dimmerL2.setPower(duty_scale);
      Serial.print("Duty scale: ");
      Serial.println(duty_scale);
      local_data[__DataTemp] = current_temperature;
      sendEvent(local_data, _ReadTemp, TxData);
      current_temperature = 0;
      num_sample = 0;
      i += 1;
      lastPIDUpdateTime = currentTime;
    }
  }
}

float control_ACDimer(double lecture_sensor, double ReferenceTemperature)
{
  float pid_output = pid.PID(lecture_sensor, ReferenceTemperature);
  float duty_scale = map(pid_output, -255, 255, 100, 0);
  return duty_scale;
}

float readTemperature()
{
  return thermocouple.readCelsius();
}

double linear_interp(double x, double x0, double x1, double y0, double y1)
{
  return (y1 - y0) / (x1 - x0) * (x - x0) + y0;
}

void calculate_reflow_profile(double temp_reflow_individual[360], double local_data[13])
{
  int TimeCooling = 360;
  double profile_time[5] = {0, local_data[__TimePreheat], local_data[__TimeSoak], local_data[__TimePeak], (double)TimeCooling};
  double profile_temp[5];

  profile_temp[0] = 20.0;
  profile_temp[1] = local_data[__Preheat];
  profile_temp[2] = local_data[__Soak];
  profile_temp[3] = local_data[__Peak];
  profile_temp[4] = local_data[__Cooling];

  for (int i = 0; i < 360; i++)
  {
    if (i >= profile_time[3])
    {
      temp_reflow_individual[i] = linear_interp(i, profile_time[3], profile_time[4],
                                                profile_temp[3], profile_temp[4]);
    }
    else if (i >= profile_time[2])
    {
      temp_reflow_individual[i] = linear_interp(i, profile_time[2], profile_time[3],
                                                profile_temp[2], profile_temp[3]);
    }
    else if (i >= profile_time[1])
    {
      temp_reflow_individual[i] = linear_interp(i, profile_time[1], profile_time[2],
                                                profile_temp[1], profile_temp[2]);
    }
    else if (i >= profile_time[0])
    {
      temp_reflow_individual[i] = linear_interp(i, profile_time[0], profile_time[1],
                                                profile_temp[0], profile_temp[1]);
    }
    else
    {
      temp_reflow_individual[i] = 20.0;
    }
  }
}

void pinRequestInterrupt()
{
  digitalWrite(INTERRUPT_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(INTERRUPT_PIN, LOW);
  Serial.println("pinRequestInterrupt");
}

void receiveEvent(int _numBytes)
{
  if (Wire.available() && _numBytes == sizeof(RxData))
  {
    Wire.readBytes((byte *)&RxData, sizeof(RxData));
  }
  _command_ = RxData.command;
  Serial.println("command: ");
  Serial.println(RxData.command);
  Serial.print(" - data: ");
  for (int i = 0; i < 13; i++)
  {
    local_data[i] = RxData.data[i];
    Serial.print(RxData.data[i]);
    Serial.print(" ");
  }
}

void sendEvent(double data[13], int _command_, I2CData &Txdata)
{
  Txdata.command = _command_;
  for (int i = 0; i < 13; i++)
  {
    Txdata.data[i] = local_data[i];
  }
  pinRequestInterrupt();
}

void requestEvent()
{
  Wire.write((byte *)&TxData, sizeof(TxData));
}

void HandShake()
{
  local_data[__DataTemp] = _HandShakeMssage;
  sendEvent(local_data, _HandShakeCmmd, TxData);
  Serial.println("HandShake ");
}