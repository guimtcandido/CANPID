# CANPID


The best library for manual PID control, it is easy and straight forward to use!

There is an example to use with arduino and proteus, make sure you have the arduino library for proteus, the proteus file has an website of it in the board

Initialize:

class PID Test_PID(1024, 0);  // Set a class for the use and give it the MAX and Minimium values for control

Mandatory Calls:

// Set gains for the control system, if the acquisition time isn't the same over the time, you can set the delay time which the control will adapt to it

Test_PID.ParamSet(Kp, Ki, Kd, ti, td); 

// Update the current error on the control algorithm for signal calculation

Test_PID.updtError(Setpoint, currentVariable); 

// This call exists and must be called before applying the control since it is the one that calculates the control and returns wether the control is saturated MAX, MIN or in range  ---->     0 -> Saturated Min;   1 -> In range;   2 -> Saturated Max;

Test_PID.OutSignal(); 

//Returns Output Control Value

Test_PID.Control();


Systems Analysis Calls:

//This calls are very usefull to understand the behaviour of the systems indepentely and helps on tuning

//Returns Error

float getError();

//Returns Integral Error

float getIntegral();

//Returns Diferential Error

float getDiferential();
