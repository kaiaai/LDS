// Based on
//   Arduino PID Library - Version 1.0.1
//   by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
//
// Copyright 2023-2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

class PID_v1 {
  public:

    //Constants used in some of the functions below
    static const int8_t AUTOMATIC = 1;
    static const int8_t MANUAL = 0;
    static const int8_t DIRECT = 0;
    static const int8_t REVERSE = 1;

    //commonly used functions **************************************************************************
    PID_v1();
    void init(float*, float*, float*,  // * links the PID to the Input, Output, and 
         float, float, float, int);    //   Setpoint.  Initial tuning parameters are also set here
  
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(float, float); //clamps the output to a specific range. 0-255 by default, but
                      //it's likely the user will want to change this depending on
                      //the application
  


  //available but not commonly used functions ********************************************************
    void SetTunings(float, float,       // * While most users will set the tunings once in the 
                    float);             //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetControllerDirection(int);   // * Sets the Direction, or "Action" of the controller. DIRECT
                      //   means the output will increase when error is positive. REVERSE
                      //   means the opposite.  it's very unlikely that this will be needed
                      //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
                      
    //Display functions ****************************************************************
    float GetKp();              // These functions query the pid for interal values.
    float GetKi();              //  they were created mainly for the pid front-end,
    float GetKd();              // where it's important to know what is actually 
    int GetMode();              //  inside the PID.
    int GetDirection();           //

  private:
    void Initialize();
    
    float dispKp;       // * we'll hold on to the tuning parameters in user-entered 
    float dispKi;       //   format for display purposes
    float dispKd;       //
      
    float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter
  
    int controllerDirection;
  
    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the 
    float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
          
    unsigned long lastTime;
    float ITerm, lastInput;
  
    unsigned long SampleTime;
    float outMin, outMax;
    bool inAuto;
};
