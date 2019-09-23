#pragma once
#include "mbed.h"

struct
{
  PinName XAxisAPulse = PD_14;
  PinName XAxisBPulse = PD_15;
  PinName XAxisIndexPulse = NC;
  PinName YAxisAPulse = PF_12;
  PinName YAxisBPulse = PF_13;
  PinName YAxisIndexPulse = NC;
} OdometryPin;

struct
{
  PinName LCD1TX = PE_1;
  PinName LCD1RX = PE_0;
  PinName UIFTX = PC_10; //user interface
  PinName UIFRX = PC_11;
  PinName MDD1TX = PD_5;
  PinName MDD1RX = PD_6;
  PinName MDD2TX = PB_6;
  PinName MDD2RX = PB_15;
  PinName MDD3TX = PC_6;
  PinName MDD3RX = PC_7;
} serialDevice;

struct
{
  PinName toBegin = PE_15;
  PinName frontR = PE_12;
  PinName frontL = PE_10;
  PinName sideR = PE_14;
  PinName sideL = PE_15;
  PinName rojarBottomR = PE_7;
  PinName rojarBottomL = PE_8;
} Switch;

struct
{
  PinName IMUSDA = PB_11;
  PinName IMUSCL = PB_10;
} I2CPin;

struct
{
  PinName clothHangRightEncoderAPulse = PC_2;
  PinName clothHangRightEncoderBPulse = PB_1;
  PinName clothHangLeftEncoderAPulse = PD_3;
  PinName clothHangLeftEncoderBPulse = PD_4;
  PinName rojarArmRightEncoderAPulse = PB_8;
  PinName rojarArmRightEncoderBPulse = PB_9;
  PinName rojarArmLeftEncoderAPulse = PA_6;
  PinName rojarArmLeftEncoderBPulse = PA_5;
  PinName holderRightServoR = PF_8; //pf6
  PinName holderRightServoL = PF_7; //pf7
  PinName holderLeftServoR = PE_6;
  PinName holderLeftServoL = PE_5;
} MecaPin;
