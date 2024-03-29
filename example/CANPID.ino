#include <Arduino.h>
#include "CANPID.h"

//Example for use the PID library
//This is the implementation principle of the library for any use (AVR,PIC,STM32,ESP32,PC Platforms,....)

unsigned long prevTime = 0;

class PID myPID(0, 255);

void setup()
{

    Serial.begin(9600);

    myPID.ParamSet(1, 0.1, 0, 1, 1); // Set the PID parameters & time Constants
}

void loop()
{
    float tempVoltage = (float)(analogRead(A0) * 5.00) / 1023.00; // Read the analog input and convert it to voltage
    float temperature = tempVoltage * 100;                        // Convert the voltage to temperature

    myPID.updtError(100, temperature); // Compute Error ---> Setpoint = 100, temperature = temperature
    myPID.OutSignal();                 // Compute the control signal

    analogWrite(9, myPID.Control()); // Write the control signal to the PWM pin (9)

    if (millis() - prevTime > 500)
    {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" Setpoint: ");
        Serial.print(100);
        Serial.print(" Control Signal: ");
        Serial.println(myPID.Control());
        prevTime = millis();
    }
}