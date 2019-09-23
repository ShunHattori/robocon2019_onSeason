#pragma once
#include "mbed.h"
#include "mbeddedPin.hpp"

struct baudRate
{
  const long HardwareSerial = 256000;
  const int LCD = 9600;
  const long I2C = 400000;
  const long SoftwareSerial = 115200;
} SerialBaud;

struct parameter
{
  const int encoderPPRLow = 48;
  const int encoderPPRHigh = 100;
  const double encoderAttachedWheelRadius = 2.54; //mini(50) omni
  const double permitErrorCircleRadius = 6.0;
  const double driveDisableRadius = 1.5;
  const int decreaseSpeedCircleRadius = 70;
  const double estimateDriveMaxPWM = 0.5; // max:0.7, recommend:0.64 //DEFAULT 0.5
  const double estimateDriveMinPWM = 0.11;
  const double estimatePegMaxPWM = 0.45;
  const double estimateHangerMaxPWM = 0.6;
  const double estimateRojarArmMaxPWM = 0.75;
  const double PegVoltageImpressionTime = 0.35;
} Robot;

struct CoordBias
{
  const int XAxis = 0; //全体を5cm左に動かす
  const int Yaxis = 0;
} FieldBias;

static int currentRunningMode, whichMecha;
static unsigned int wayPointSignature = 1;

double driverPWMOutput[3];
static double pegAttacherPWM[2][2];                             //right(CW,CCW),left(CW,CCW)
static double hangerPWM[2][2];                                  //right(CW,CCW),left(CW,CCW)
static double rojarArmPWM[2][2];                                //right(CW,CCW),left(CW,CCW)
Serial STLinkTerminal(USBTX, USBRX, SerialBaud.HardwareSerial); //Surfaceのターミナルとの通信用ポート
Serial serialLCD(serialDevice.LCD1TX, serialDevice.LCD1RX, SerialBaud.LCD);
NewHavenDisplay LCDDriver(serialLCD);
Timer TimerForQEI;                  //エンコーダクラス用共有タイマー
Timer TimerForLCD;                  //LCD更新用タイマー
Timer clothHangerTimer;             //機構シーケンス用タイマー
DigitalOut modeIndicatorLED1(LED1); //一時的モード切替表示用LED
DigitalOut modeIndicatorLED2(LED2); //一時的モード切替表示用LED
DigitalOut modeIndicatorLED3(LED3); //一時的モード切替表示用LED
MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, SerialBaud.I2C);
UnitProtocol UIF(serialDevice.UIFTX, serialDevice.UIFRX, SerialBaud.SoftwareSerial);
enum MDDName
{
  Drive,
  Meca1,
  Meca2,
} MDD;
UnitProtocol MDDSlave[3] = {
    UnitProtocol(serialDevice.MDD1TX, serialDevice.MDD1RX, SerialBaud.HardwareSerial),
    UnitProtocol(serialDevice.MDD2TX, serialDevice.MDD2RX, SerialBaud.HardwareSerial),
    UnitProtocol(serialDevice.MDD3TX, serialDevice.MDD3RX, SerialBaud.HardwareSerial),
};
DebounceSwitch onBoardSwitch(USER_BUTTON, DebounceSwitch::PULLDOWN);
DebounceSwitch startButton(Switch.toBegin, DebounceSwitch::PULLUP); //create object using pin "PG_2" with PullUpped
enum set
{
  right,
  left,
} mechaSet;
DebounceSwitch limitSwitchBar[2]{
    DebounceSwitch(Switch.frontR, DebounceSwitch::PULLUP),
    DebounceSwitch(Switch.frontL, DebounceSwitch::PULLUP),
};
DebounceSwitch limitSwitchSide[2]{
    DebounceSwitch(Switch.sideR, DebounceSwitch::PULLUP),
    DebounceSwitch(Switch.sideL, DebounceSwitch::PULLUP),
};
DebounceSwitch limitSwitchRojarArm[2]{
    DebounceSwitch(Switch.rojarBottomR, DebounceSwitch::PULLUP),
    DebounceSwitch(Switch.rojarBottomL, DebounceSwitch::PULLUP),
};
ClothHold holder[2] = {
    ClothHold(MecaPin.holderRightServoR, MecaPin.holderRightServoL), //right,leftServo
    ClothHold(MecaPin.holderLeftServoR, MecaPin.holderLeftServoL),   //right,leftServo
};
Peg pegAttacher[2]{
    Peg(Robot.estimatePegMaxPWM, Robot.PegVoltageImpressionTime, pegAttacherPWM[0]), //pwm, time
    Peg(Robot.estimatePegMaxPWM, Robot.PegVoltageImpressionTime, pegAttacherPWM[1]), //pwm, time
};
ClothHang hanger[2]{
    ClothHang(hangerPWM[right]),
    ClothHang(hangerPWM[left]),
};
RojarArm rojarArm[2]{
    RojarArm(rojarArmPWM[right], limitSwitchRojarArm[right]),
    RojarArm(rojarArmPWM[left], limitSwitchRojarArm[left]),
};
QEI clothHangEncoder[2]{
    QEI(MecaPin.clothHangRightEncoderAPulse, MecaPin.clothHangRightEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING),
    QEI(MecaPin.clothHangLeftEncoderAPulse, MecaPin.clothHangLeftEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING),
};
QEI rojarArmEncoder[2]{
    QEI(MecaPin.rojarArmRightEncoderAPulse, MecaPin.rojarArmRightEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING),
    QEI(MecaPin.rojarArmLeftEncoderAPulse, MecaPin.rojarArmLeftEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING),
};

QEI encoderXAxis(OdometryPin.XAxisAPulse, OdometryPin.XAxisBPulse, OdometryPin.XAxisIndexPulse, Robot.encoderPPRHigh, &TimerForQEI, QEI::X4_ENCODING);
QEI encoderYAxis(OdometryPin.YAxisAPulse, OdometryPin.YAxisBPulse, OdometryPin.YAxisIndexPulse, Robot.encoderPPRHigh, &TimerForQEI, QEI::X4_ENCODING);
MWodometry odometryXAxis(encoderXAxis, Robot.encoderPPRHigh, Robot.encoderAttachedWheelRadius);
MWodometry odometryYAxis(encoderYAxis, Robot.encoderPPRHigh, Robot.encoderAttachedWheelRadius);
LocationManager<double> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometryXAxis, odometryYAxis, IMU, Robot.permitErrorCircleRadius, Robot.driveDisableRadius, Robot.decreaseSpeedCircleRadius);
OmniKinematics3WD OmniKinematics;