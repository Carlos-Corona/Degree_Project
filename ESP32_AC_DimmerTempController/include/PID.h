#include <Arduino.h>

#ifndef __PIDCONTROLLER_H__ // Evita conflictos de definición
#define __PIDCONTROLLER_H__

#include <PID.h>

class PIDController
{
private:
  // Constantes del controlador PID.
  float Kp;
  float Ki;
  float Kd;
  // Variable Global que almacena el error previo
  float prev_Error;
  // Tiempo de muestreo en segundos.
  float T;

public:
  /**
   * Constructor de la clase PIDController.
   * @param Kp Constante proporcional del controlador PID.
   * @param Ki Constante integral del controlador PID.
   * @param Kd Constante derivativa del controlador PID.
   * @param prev_Error Valor inicial del error anterior.
   * @param T Tiempo de muestreo del controlador PID en segundos.
   */
  PIDController(float Kp, float Ki, float Kd, float prev_Error, float T)
  {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->prev_Error = prev_Error;
    this->T = T;
  }

  /**
   * Calcula el valor del controlador PID para alcanzar una temperatura deseada.
   * @param Sensor Valor actual del sensor de temperatura.
   * @param referenceTemp Temperatura deseada.
   * @return Valor del controlador PID.
   */
  float PID(float Sensor, float referenceTemp)
  {
    // Variable para almacenar el valor de la integral del error.
    static float Integral = 0;

    // Calcular el valor del error actual.
    float Error = Sensor - referenceTemp;

    // Calcular el término proporcional del controlador PID.
    float Proporcional = Error * Kp;

    // Control anti-windup integral.
    // Error máximo para que pueda funcionar el término integral.
    float max_integral_error = 20;
    if (abs(Error) > max_integral_error)
    {
      Integral = 0;
    }
    else
    {
      // Calcular el término integral del controlador PID.
      Integral = Integral + Error * Ki * T;
    }

    // Calcular el término derivativo del controlador PID.
    float Derivativo = (Error - prev_Error) * Kd / T;

    // Calcular el valor del controlador PID.
    float Control = Proporcional + Integral + Derivativo;

    // Actualizar el valor del error anterior.
    prev_Error = Error;
    if (Control > 255)
    {
      return 255;
    }
    else if (Control < -255)
    {
      return -255;
    }
    return Control;
  }
  void SetPIDTunning(float Kp, float Ki, float Kd)
  {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->prev_Error = 0;
  }
};

#endif // __PIDCONTROLLER_H__