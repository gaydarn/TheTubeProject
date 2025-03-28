#include "Potentiometer.h"

#include "mbed.h"
using namespace std;

Potentiometer::Potentiometer(byte potValuePin)
{
  _potValuePin = potValuePin;
  analogReadResolution(12);
}

float Potentiometer::getValue()
{
  return float(analogRead(_potValuePin));
}

float Potentiometer::getValueProp()
{
  float sensorValue = this->getValue();

  //Bornage des valeurs du potentiomètre pour avoir une plage à 0% et une plage à 100%
  sensorValue = ((sensorValue < ANALOG_VAL_0) ? 0 : sensorValue - ANALOG_VAL_0);
  sensorValue = ((sensorValue > ANALOG_VAL_100) ? ANALOG_VAL_100 - ANALOG_VAL_0 : sensorValue);

  float propValue = (sensorValue/(ANALOG_VAL_100 - ANALOG_VAL_0));

  return 1-propValue; //Potentiomètre inversé, on corrige pour avoir 0% à gauche et 100% à droite
}

float Potentiometer::getValuePercent()
{
  return this->getValueProp()*100;
}

