#include "CommunicationSource\UnitProtocol.hpp"
#include "DriveSource\DriveTrain.h"
#include "DriveSource\LocationManager.h"
#include "DriveSource\MWodometry.h"
#include "DriveSource\OmniKinematics3WD.h"
#include "DriveSource\TimeIncreaser.h"
#include "MechanismSource\ClothHang.h"
#include "MechanismSource\ClothHold.h"
#include "MechanismSource\Peg.h"
#include "MechanismSource\RojarArm.h"
#include "MechanismSource\Servo.h"
#include "RobotParameters.hpp"
#include "SensorSource\DebounceSwitch.h"
#include "SensorSource\MPU9250.h"
#include "SensorSource\QEI.h"
#include "mbed.h"
#include "mbeddedPin.hpp"

#define GAME_MODECHANGE_ONBOARDSWITCH //基板上の青スイッチで動作切り替え（シーツ・バスタオル）
//#define BUDEGGER
//#define CONTROLPANEL
//#define CONTROLPANEL_GETMODE_TEST
//#define TEST_DRIVE
//#define TEST_RojarArm_UpDownLoop
//#define TEST_PEGLaunch      //tested on 6/20(Thur)
//#define TEST_HangerMovement //tested on 6/20(Thur)
//#define TEST_HoldServo
//#define TEST_RojarArmUpDownOnce

void allUpdate();
bool getRobotModeWithSwitch();

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
  PinName holderRightServoR = PF_7; //pf6
  PinName holderRightServoL = PF_8; //pf7
  PinName holderLeftServoR = PE_5;
  PinName holderLeftServoL = PE_6;
} MecaPin;

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
  const double driveDisableRadius = 1.3;
  const int decreaseSpeedCircleRadius = 70;
  const double estimateDriveMaxPWM = 0.5; // max:0.7, recommend:0.64 //DEFAULT 0.5
  const double estimateDriveMinPWM = 0.10;
  const double estimatePegMaxPWM = 0.45;
  const double estimateHangerMaxPWM = 0.5;
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
static double pegAttacherPWM[2][2]; //right(CW,CCW),left(CW,CCW)
static double hangerPWM[2][2];      //right(CW,CCW),left(CW,CCW)
static double rojarArmPWM[2][2];    //right(CW,CCW),left(CW,CCW)
bool whichServo;

Serial STLinkTerminal(USBTX, USBRX, SerialBaud.HardwareSerial); //Surfaceのターミナルとの通信用ポート
Timer TimerForQEI;                                              //エンコーダクラス用共有タイマー
Timer clothHangerTimer;                                         //機構シーケンス用タイマー
Timer pidLoopTimer;
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

enum gameMode
{
  RED_FRONT_PRE_bathTowelLeft,
  RED_NONE,
  RED_MIDDLE_PRE_bathTowelLeft,
  RED_MIDDLE_PRE_bathTowelRight,
  RED_MIDDLE_PRE_bathTowelboth,
  RED_MIDDLE_FINAL_bathTowelLeft,
  RED_MIDDLE_FINAL_bathTowelRight,
  RED_MIDDLE_FINAL_bathTowelboth,
  RED_BACK_PRE_Sheets,
  RED_BACK_FINAL_Sheets,
  BLUE_FRONT_PRE_bathTowelRight,
  BLUE_NONE,
  BLUE_MIDDLE_PRE_bathTowelRight,
  BLUE_MIDDLE_PRE_bathTowelLeft,
  BLUE_MIDDLE_PRE_bathTowelboth,
  BLUE_MIDDLE_FINAL_bathTowelRight,
  BLUE_MIDDLE_FINAL_bathTowelLeft,
  BLUE_MIDDLE_FINAL_bathTowelboth,
  BLUE_BACK_PRE_Sheets,
  BLUE_BACK_FINAL_Sheets,
};

void updateLimitSwitchBar();
bool getLimitSwitchBarStats();
void driveAutoConverger();

int main(void)
{
  accelAlgorithm.setCurrentXPosition((double)FieldBias.XAxis);
  for (int i = 0; i < 2; i++)
  {
    hanger[i].setMaxPWM(Robot.estimateHangerMaxPWM);
    rojarArm[i].setMaxPWM(Robot.estimateRojarArmMaxPWM);
    holder[i].free(right);
    holder[i].free(left);
  }
  accelAlgorithm.setMaxOutput(Robot.estimateDriveMaxPWM);
  accelAlgorithm.setMinOutput(Robot.estimateDriveMinPWM);
  OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
#ifdef GAME_MODECHANGE_ONBOARDSWITCH

  IMU.setup();
  while (1)
  {
    /*if (getRobotModeWithSwitch())
      break;*/
    uint8_t UIFData[1];
    if (UIF.receive(UIFData))
    {
      currentRunningMode = UIFData[0];
      STLinkTerminal.printf("MODE RECEIVED. %d\r\n", currentRunningMode);
      break;
    }
  }
  switch (currentRunningMode)
  {
    case RED_FRONT_PRE_bathTowelLeft:
      whichMecha = left;
      whichServo = left;
      robotLocation.addPoint((117), -(200));       //近づいてリミット監視開始 初期位置ずれてるから長めになってる！
      robotLocation.addPoint((203), -(190));       //竿目印のところまで移動(ちょっと後ろ目)
      robotLocation.addPoint((203), -(190));       //ハサミつく位置まで前に
      robotLocation.addPoint((203), -(190));       //最初に直進 //198
      robotLocation.addPoint((203 + 100), -(200)); //洗濯物横に引っ張る //202
      robotLocation.addPoint((203 + 100), -(190)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((203 + 100), -(170));
      robotLocation.addPoint(30, 30); //初期位置
      break;
    case RED_MIDDLE_PRE_bathTowelLeft:
      whichMecha = left;
      whichServo = left;
      robotLocation.addPoint(10, -(390));         //二本目のポール前
      robotLocation.addPoint((117), -(395));      //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((193), -(380));      //竿目印のところまで移動
      robotLocation.addPoint((193), -(390));      //ハサミつく位置まで前に
      robotLocation.addPoint((193), -(390));      //最初に直進
      robotLocation.addPoint((193 + 40), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint((193 + 40), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((193 + 40), -(370));
      robotLocation.addPoint(30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);     //初期位置
      break;
    case RED_MIDDLE_PRE_bathTowelRight:
      whichMecha = right;
      whichServo = left;
      robotLocation.addPoint(10, -(390));          //二本目のポール前
      robotLocation.addPoint((117), -(395));       //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((237), -(380));       //竿目印のところまで移動
      robotLocation.addPoint((237), -(390));       //ハサミつく位置まで前に
      robotLocation.addPoint((237), -(390));       //最初に直進
      robotLocation.addPoint((237 + 100), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint((237 + 100), -(385)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((237 + 100), -(370));
      robotLocation.addPoint(30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);     //初期位置
      break;
    case RED_MIDDLE_PRE_bathTowelboth:
      whichMecha = left; //途中で変わる!!
      whichServo = left;
      robotLocation.addPoint(10, -(390));          //1二本目のポール前
      robotLocation.addPoint((117), -(395));       //2リミット接触前ちょこちょこ進む
      robotLocation.addPoint((208), -(390));       //3竿目印のところまで移動
      robotLocation.addPoint((208), -(390));       //4ハサミつく位置まで前に
      robotLocation.addPoint((208), -(390));       //5最初に直進
      robotLocation.addPoint((208 + 40), -(400));  //6洗濯物横に引っ張る
      robotLocation.addPoint((208 + 40), -(385));  //7ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((257), -(390));       //8竿目印のところまで移動     2つ目！
      robotLocation.addPoint((257), -(390));       //9ハサミつく位置まで前に
      robotLocation.addPoint((257 + 100), -(400)); //10洗濯物横に引っ張る
      robotLocation.addPoint((257 + 100), -(385)); //11ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((257 + 100), -(370)); //12
      robotLocation.addPoint(30, -(335));          //13直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);              //14初期位置
      break;
    case RED_MIDDLE_FINAL_bathTowelLeft:
      whichMecha = left;
      whichServo = left;
      robotLocation.addPoint(10, -(390));          //二本目のポール前
      robotLocation.addPoint((117), -(395));       //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((180), -(380));       //竿目印のところまで移動
      robotLocation.addPoint((180), -(390));       //ハサミつく位置まで前に
      robotLocation.addPoint((180), -(390));       //最初に直進
      robotLocation.addPoint((180 + 100), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint((180 + 100), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((180 + 100), -(370));
      robotLocation.addPoint(30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);     //初期位置
      break;
    case RED_MIDDLE_FINAL_bathTowelRight:
      whichMecha = right;
      whichServo = left;
      robotLocation.addPoint(10, -(390));          //二本目のポール前
      robotLocation.addPoint((117), -(395));       //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((275), -(380));       //竿目印のところまで移動
      robotLocation.addPoint((275), -(390));       //ハサミつく位置まで前に
      robotLocation.addPoint((275), -(390));       //最初に直進
      robotLocation.addPoint((275 + 100), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint((275 + 100), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((275 + 100), -(370));
      robotLocation.addPoint(30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);     //初期位置
      break;
    case RED_MIDDLE_FINAL_bathTowelboth:
      whichMecha = left; //途中で変わる!!
      whichServo = left;
      robotLocation.addPoint(10, -(390));          //1二本目のポール前
      robotLocation.addPoint((117), -(395));       //2リミット接触前ちょこちょこ進む
      robotLocation.addPoint((193), -(390));       //3竿目印のところまで移動
      robotLocation.addPoint((193), -(390));       //4ハサミつく位置まで前に
      robotLocation.addPoint((193), -(390));       //5最初に直進
      robotLocation.addPoint((193 + 40), -(400));  //6洗濯物横に引っ張る
      robotLocation.addPoint((193 + 40), -(385));  //7ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((237), -(390));       //8竿目印のところまで移動     2つ目！
      robotLocation.addPoint((237), -(390));       //9ハサミつく位置まで前に
      robotLocation.addPoint((237 + 100), -(400)); //10洗濯物横に引っ張る
      robotLocation.addPoint((237 + 100), -(385)); //11ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((237 + 100), -(370)); //12
      robotLocation.addPoint(30, -(335));          //13直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);              //14初期位置
      break;
    case RED_BACK_PRE_Sheets:
    case RED_BACK_FINAL_Sheets:
      whichMecha = right;
      whichServo = left;
      robotLocation.addPoint(10, -(520));
      robotLocation.addPoint((133), -(570));
      robotLocation.addPoint((170), -(590));
      robotLocation.addPoint((340), -(600)); //右リミットスイッチが接触しないから596に変更
      robotLocation.addPoint((375), -(575));
      robotLocation.addPoint(40, -(520));
      robotLocation.addPoint(25, 20);
      break;
    case BLUE_FRONT_PRE_bathTowelRight:
      whichMecha = right;
      whichServo = right;
      robotLocation.addPoint(-(117), -(200));       //近づいてリミット監視開始 初期位置ずれてるから長めになってる！
      robotLocation.addPoint(-(203), -(190));       //竿目印のところまで移動(ちょっと後ろ目)
      robotLocation.addPoint(-(203), -(190));       //ハサミつく位置まで前に
      robotLocation.addPoint(-(203), -(190));       //最初に直進 //198
      robotLocation.addPoint(-(203 + 100), -(200)); //洗濯物横に引っ張る //202
      robotLocation.addPoint(-(203 + 100), -(190)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(-(203 + 100), -(170));
      robotLocation.addPoint(-30, 30); //初期位置
      break;
    case BLUE_MIDDLE_PRE_bathTowelRight:
      whichMecha = right;
      whichServo = right;
      robotLocation.addPoint(-10, -(390));         //二本目のポール前
      robotLocation.addPoint(-(117), -(395));      //リミット接触前ちょこちょこ進む
      robotLocation.addPoint(-(193), -(380));      //竿目印のところまで移動
      robotLocation.addPoint(-(193), -(390));      //ハサミつく位置まで前に
      robotLocation.addPoint(-(193), -(390));      //最初に直進
      robotLocation.addPoint(-(193 + 40), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint(-(193 + 40), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(-(193 + 40), -(370));
      robotLocation.addPoint(-30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(-30, 30);     //初期位置
      break;
    case BLUE_MIDDLE_PRE_bathTowelLeft:
      whichMecha = left; //バスタオル1.5m縦掛け
      whichServo = right;
      robotLocation.addPoint(-10, -(390));          //二本目のポール前
      robotLocation.addPoint(-(117), -(395));       //リミット接触前ちょこちょこ進む
      robotLocation.addPoint(-(237), -(380));       //竿目印のところまで移動
      robotLocation.addPoint(-(237), -(390));       //ハサミつく位置まで前に
      robotLocation.addPoint(-(237), -(390));       //最初に直進
      robotLocation.addPoint(-(237 + 103), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint(-(237 + 103), -(385)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(-(237 + 103), -(370));
      robotLocation.addPoint(-30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(-30, 30);     //初期位置
      break;
    case BLUE_MIDDLE_PRE_bathTowelboth:
      whichMecha = right; //途中で変わる!!
      whichServo = right;
      robotLocation.addPoint(-10, -(390));          //1二本目のポール前
      robotLocation.addPoint(-(117), -(395));       //2リミット接触前ちょこちょこ進む
      robotLocation.addPoint(-(193), -(390));       //3竿目印のところまで移動
      robotLocation.addPoint(-(193), -(390));       //4ハサミつく位置まで前に
      robotLocation.addPoint(-(193), -(390));       //5最初に直進
      robotLocation.addPoint(-(193 + 40), -(400));  //6洗濯物横に引っ張る
      robotLocation.addPoint(-(193 + 40), -(385));  //7ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(-(237), -(390));       //8竿目印のところまで移動     2つ目！
      robotLocation.addPoint(-(237), -(390));       //9ハサミつく位置まで前に
      robotLocation.addPoint(-(237 + 103), -(400)); //10洗濯物横に引っ張る
      robotLocation.addPoint(-(237 + 103), -(385)); //11ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(-(237 + 103), -(370)); //12
      robotLocation.addPoint(-30, -(335));          //13直線移動できる位置まで戻ってくる
      robotLocation.addPoint(-30, 30);              //14初期位置
      break;
    case BLUE_MIDDLE_FINAL_bathTowelRight:
      whichMecha = right;
      whichServo = right;
      robotLocation.addPoint(-10, -(390));          //二本目のポール前
      robotLocation.addPoint(-(117), -(395));       //リミット接触前ちょこちょこ進む
      robotLocation.addPoint(-(180), -(380));       //竿目印のところまで移動
      robotLocation.addPoint(-(180), -(390));       //ハサミつく位置まで前に
      robotLocation.addPoint(-(180), -(390));       //最初に直進
      robotLocation.addPoint(-(180 + 100), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint(-(180 + 100), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(-(180 + 100), -(370));
      robotLocation.addPoint(-30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(-30, 30);     //初期位置
      break;
    case BLUE_MIDDLE_FINAL_bathTowelLeft:
      whichMecha = left;
      whichServo = right;
      robotLocation.addPoint(-10, -(390));          //二本目のポール前
      robotLocation.addPoint(-(117), -(395));       //リミット接触前ちょこちょこ進む
      robotLocation.addPoint(-(275), -(380));       //竿目印のところまで移動
      robotLocation.addPoint(-(275), -(390));       //ハサミつく位置まで前に
      robotLocation.addPoint(-(275), -(390));       //最初に直進
      robotLocation.addPoint(-(275 + 100), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint(-(275 + 100), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(-(275 + 100), -(370));
      robotLocation.addPoint(-30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(-30, 30);     //初期位置
      break;
    case BLUE_MIDDLE_FINAL_bathTowelboth:
      whichMecha = right; //途中で変わる!!
      whichServo = right;
      robotLocation.addPoint(-10, -(390));          //1二本目のポール前
      robotLocation.addPoint(-(117), -(395));       //2リミット接触前ちょこちょこ進む
      robotLocation.addPoint(-(180), -(390));       //3竿目印のところまで移動
      robotLocation.addPoint(-(180), -(390));       //4ハサミつく位置まで前に
      robotLocation.addPoint(-(180), -(390));       //5最初に直進
      robotLocation.addPoint(-(180 + 100), -(400)); //6洗濯物横に引っ張る
      robotLocation.addPoint(-(180 + 100), -(390)); //7ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(-(275), -(390));       //8竿目印のところまで移動     2つ目！
      robotLocation.addPoint(-(275), -(390));       //9ハサミつく位置まで前に
      robotLocation.addPoint(-(275 + 100), -(400)); //10洗濯物横に引っ張る
      robotLocation.addPoint(-(275 + 100), -(385)); //11ロジャー降ろしながら後ろに引く
      robotLocation.addPoint(-(275 + 100), -(370)); //12
      robotLocation.addPoint(-30, -(335));          //13直線移動できる位置まで戻ってくる
      robotLocation.addPoint(-30, 30);              //14初期位置
      break;
    case BLUE_BACK_PRE_Sheets:
    case BLUE_BACK_FINAL_Sheets:
      whichMecha = left;
      whichServo = right;
      robotLocation.addPoint(-10, -(520));
      robotLocation.addPoint(-(133), -(570));
      robotLocation.addPoint(-(170), -(590));
      robotLocation.addPoint(-(340), -(600)); //右リミットスイッチが接触しないから596に変更
      robotLocation.addPoint(-(375), -(575));
      robotLocation.addPoint(-40, -(520));
      robotLocation.addPoint(-25, 20);
      break;
  }
  for (int i = 0; i < 2; i++)
  {
    holder[i].grasp(right);
    holder[i].grasp(left);
  }
  //holder[whichMecha].setFieldMode(9 < currentRunningMode ? 1 : 0); //10以上
  //holder[whichMecha].grasp(!whichServo);
  //holder[!whichMecha].grasp(whichServo);
  robotLocation.sendNext();
  accelAlgorithm.setPositionChangedFlag();
  accelAlgorithm.setAllocateErrorCircleRadius(40);
  while (1)
  {
    STLinkTerminal.printf("%.1lf %.1lf %.1lf   \r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
    //STLinkTerminal.printf("%.3lf\t%.3lf\t%.3lf\r\n", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
    //STLinkTerminal.printf("%d\t%d\t%d\t%d\t%d\t%d\t\r\n", (int)(pegAttacherPWM[0][0] * 100), (int)(pegAttacherPWM[0][1] * 100), (int)(hangerPWM[0][0] * 100), (int)(hangerPWM[0][1] * 100), (int)(rojarArmPWM[0][0] * 100), (int)(rojarArmPWM[0][1] * 100));
    //STLinkTerminal.printf("%d\t%d\t%d\t\r\n", (int)(driverPWMOutput[0] * 100), (int)(driverPWMOutput[1] * 100), (int)(driverPWMOutput[2]));
    //updateDisplayDatas();
    allUpdate();
    //driveAutoConverger();
    switch (currentRunningMode)
    {
      case RED_FRONT_PRE_bathTowelLeft:
      case BLUE_FRONT_PRE_bathTowelRight:
        accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
        updateLimitSwitchBar();
        if (getLimitSwitchBarStats() && wayPointSignature == 1)
        {
          accelAlgorithm.setCurrentYPosition(-(190));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(190), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
            OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
            robotLocation.sendNext();
            accelAlgorithm.setPositionChangedFlag();
            accelAlgorithm.update();
            wayPointSignature++;
          }
          flag++;
        }
        if (accelAlgorithm.getStats())
        {
          switch (wayPointSignature)
          {
            case 1:
              OmniKinematics.setMaxPWM(0.12);
              accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
              rojarArm[whichMecha].setHeight(300);
              updateLimitSwitchBar();
              if (!getLimitSwitchBarStats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 20, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 2:
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 3:
              static int armPhase = 1, hangerHasDoneFlag = 0;    //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhase == 1) //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1020); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                  rojarArm[whichMecha].setHeight(0);
                }
              }
              if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats() && hangerHasDoneFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holder[whichMecha].release(whichServo);
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp(whichServo);
                  seq = 3;
                }
                if (2200 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  rojarArm[whichMecha].setHeight(200);
                }
              }
              if (armPhase == 2)
              {
                if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    accelAlgorithm.setPositionChangedFlag();
                    rojarArm[whichMecha].setHeight(560); //バスタオル後ろ巻いてしまうからめっちゃ高めに上げる　前：600
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 4:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 5:
              OmniKinematics.setMaxPWM(0.17);
              robotLocation.sendNext(); //横移動
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 6:
              static bool initialFlag = 1, releasedFlag = 0;
              if (initialFlag)
              {
                OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
                clothHangerTimer.start();
                initialFlag = 0;
              }
              if (clothHangerTimer.read_ms() > 300)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setHeight(120);
                rojarArm[whichMecha].setMaxPWM(0.35);
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlag = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlag)
              {
                holder[whichMecha].half(!whichServo);
                wayPointSignature++;
                clothHangerTimer.start();
              }
              break;
            case 7:
              if (650 < clothHangerTimer.read_ms())
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                wayPointSignature++;
                clothHangerTimer.stop();
                clothHangerTimer.reset();
              }
              break;
            case 8:
              accelAlgorithm.setDecreaseCircleRadius(200);
              rojarArm[whichMecha].setHeight(0);
              rojarArm[whichMecha].setMaxPWM(Robot.estimateRojarArmMaxPWM);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              pegAttacher[whichMecha].reload();
              break;
            case 9:
              holder[whichMecha].free(!whichServo); //右
              holder[whichMecha].free(whichServo);
              while (1)
              {
                allUpdate();
              }
              break;
            default:
              while (1)
                ;
          }
        }
        break;
      case RED_MIDDLE_PRE_bathTowelRight:
      case RED_MIDDLE_FINAL_bathTowelRight:
      case BLUE_MIDDLE_PRE_bathTowelRight:
      case BLUE_MIDDLE_FINAL_bathTowelRight:
        updateLimitSwitchBar();
        if (getLimitSwitchBarStats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
            OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
            robotLocation.sendNext();
            accelAlgorithm.setPositionChangedFlag();
            accelAlgorithm.update();
            wayPointSignature++;
          }
          flag++;
        }
        if (accelAlgorithm.getStats())
        {
          switch (wayPointSignature)
          {
            case 1:
              accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
              OmniKinematics.setMaxPWM(0.3);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(1600); //押し出す前に大幅に上げる
              wayPointSignature++;
              break;
            case 2:
              updateLimitSwitchBar();
              if (!getLimitSwitchBarStats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 20, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 3:
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 4:
              static int armPhase = 1, hangerHasDoneFlag = 0;    //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhase == 1) //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1020); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                  rojarArm[whichMecha].setHeight(1270); //洗濯物離すために大幅に下げる
                }
              }
              if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats() && hangerHasDoneFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holder[whichMecha].release(whichServo);
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp(whichServo);
                  seq = 3;
                }
                if (2200 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  rojarArm[whichMecha].setHeight(1500);
                }
              }
              if (armPhase == 2)
              {
                if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    accelAlgorithm.setPositionChangedFlag();
                    rojarArm[whichMecha].setHeight(1910); //2100
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 5:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 6:
              OmniKinematics.setMaxPWM(0.17);
              robotLocation.sendNext(); //横移動
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 7:
              static bool initialFlag = 1, releasedFlag = 0;
              if (initialFlag)
              {
                OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
                clothHangerTimer.start();
                initialFlag = 0;
              }
              if (clothHangerTimer.read_ms() > 300)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setHeight(1530);
                rojarArm[whichMecha].setMaxPWM(0.35);
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlag = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlag)
              {
                holder[whichMecha].half(!whichServo);
                wayPointSignature++;
                clothHangerTimer.start();
              }
              break;
            case 8:
              if (650 < clothHangerTimer.read_ms())
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                wayPointSignature++;
                clothHangerTimer.stop();
                clothHangerTimer.reset();
              }
              break;
            case 9:
              rojarArm[whichMecha].setHeight(0);
              rojarArm[whichMecha].setMaxPWM(Robot.estimateRojarArmMaxPWM);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 10:
              holder[whichMecha].grasp(!whichServo);
              holder[whichMecha].grasp(whichServo);
              pegAttacher[whichMecha].reload();
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 11:
              holder[whichMecha].free(!whichServo);
              holder[whichMecha].free(whichServo);
              while (1)
              {
                allUpdate();
              }
              break;
            default:
              while (1)
                ;
          }
        }
        break;
      case RED_MIDDLE_PRE_bathTowelLeft:
      case RED_MIDDLE_FINAL_bathTowelLeft:
      case BLUE_MIDDLE_PRE_bathTowelLeft:
      case BLUE_MIDDLE_FINAL_bathTowelLeft:
        updateLimitSwitchBar();
        if (getLimitSwitchBarStats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
            OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
            robotLocation.sendNext();
            accelAlgorithm.setPositionChangedFlag();
            accelAlgorithm.update();
            wayPointSignature++;
          }
          flag++;
        }
        if (accelAlgorithm.getStats())
        {
          switch (wayPointSignature)
          {
            case 1:
              accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
              OmniKinematics.setMaxPWM(0.3);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(1600); //押し出す前に大幅に上げる
              wayPointSignature++;
              break;
            case 2:
              updateLimitSwitchBar();
              if (!getLimitSwitchBarStats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 20, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 3:
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 4:
              static int armPhase = 1, hangerHasDoneFlag = 0;    //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhase == 1) //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1020); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                  rojarArm[whichMecha].setHeight(1270); //洗濯物離すために大幅に下げる
                }
              }
              if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats() && hangerHasDoneFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holder[whichMecha].release(whichServo);
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp(whichServo);
                  seq = 3;
                }
                if (3000 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  rojarArm[whichMecha].setHeight(1500);
                }
              }
              if (armPhase == 2)
              {
                if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    accelAlgorithm.setPositionChangedFlag();
                    rojarArm[whichMecha].setMaxPWM(0.4);
                    rojarArm[whichMecha].setHeight(1910); //2100
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 5:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 6:
              OmniKinematics.setMaxPWM(0.17);
              robotLocation.sendNext(); //横移動
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 7:
              static bool initialFlag = 1, releasedFlag = 0;
              if (initialFlag)
              {
                OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
                clothHangerTimer.start();
                initialFlag = 0;
              }
              if (clothHangerTimer.read_ms() > 300)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setHeight(1530);
                rojarArm[whichMecha].setMaxPWM(0.35);
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlag = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlag)
              {
                holder[whichMecha].half(!whichServo);
                wayPointSignature++;
                clothHangerTimer.start();
              }
              break;
            case 8:
              if (650 < clothHangerTimer.read_ms())
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                wayPointSignature++;
                clothHangerTimer.stop();
                clothHangerTimer.reset();
              }
              break;
            case 9:
              rojarArm[whichMecha].setHeight(0);
              rojarArm[whichMecha].setMaxPWM(Robot.estimateRojarArmMaxPWM);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 10:
              holder[whichMecha].grasp(!whichServo);
              holder[whichMecha].grasp(whichServo);
              pegAttacher[whichMecha].reload();
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 11:
              holder[whichMecha].free(!whichServo);
              holder[whichMecha].free(whichServo);
              while (1)
              {
                allUpdate();
              }
              break;
            default:
              while (1)
                ;
          }
        }
        break;
      case RED_MIDDLE_PRE_bathTowelboth:
      case RED_MIDDLE_FINAL_bathTowelboth:
      case BLUE_MIDDLE_PRE_bathTowelboth:
      case BLUE_MIDDLE_FINAL_bathTowelboth:
        updateLimitSwitchBar();
        if (getLimitSwitchBarStats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
            OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
            robotLocation.sendNext();
            accelAlgorithm.setPositionChangedFlag();
            accelAlgorithm.update();
            wayPointSignature++;
          }
          flag++;
        }
        if (accelAlgorithm.getStats())
        {
          switch (wayPointSignature)
          {
            case 1:
              accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
              OmniKinematics.setMaxPWM(0.3);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(1600);
              wayPointSignature++;
              break;
            case 2:
              updateLimitSwitchBar();
              if (!getLimitSwitchBarStats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 20, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 3:
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 4:
              static int armPhaseLeft = 1, hangerHasDoneFlagLeft = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhaseLeft == 1)  //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1020); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlagLeft)
                {
                  hangerHasDoneFlagLeft = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                  rojarArm[whichMecha].setHeight(1270); //洗濯物離すために大幅に下げる
                }
              }
              if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats() && hangerHasDoneFlagLeft)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holder[whichMecha].release(whichServo);
                  seq = 2;
                }
                if (800 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp(whichServo);
                  seq = 3;
                }
                if (1600 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhaseLeft = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  rojarArm[whichMecha].setHeight(1500);
                }
              }
              if (armPhaseLeft == 2)
              {
                if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext();
                    accelAlgorithm.setPositionChangedFlag();
                    rojarArm[whichMecha].setHeight(1910);
                    rojarArm[whichMecha].update();
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 5:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 6:
              OmniKinematics.setMaxPWM(0.15);
              robotLocation.sendNext(); //横移動 6
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 7:
              static bool initialFlag = 1, releasedFlag = 0;
              if (initialFlag)
              {
                OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
                clothHangerTimer.start();
                initialFlag = 0;
              }
              if (clothHangerTimer.read_ms() > 300)
              {
                robotLocation.sendNext(); //7
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setHeight(1530);
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlag = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlag)
              {
                holder[whichMecha].half(!whichServo);
                wayPointSignature++;
                clothHangerTimer.start();
              }
              break;
            case 8:
              if (800 < clothHangerTimer.read_ms())
              {
                robotLocation.sendNext(); //8
                accelAlgorithm.setPositionChangedFlag();
                holder[whichMecha].grasp(!whichServo);
                wayPointSignature++;
                clothHangerTimer.stop();
                clothHangerTimer.reset();
              }
              break;
            case 9:
              rojarArm[whichMecha].setHeight(0);
              rojarArm[whichMecha].setMaxPWM(Robot.estimateRojarArmMaxPWM);
              rojarArm[whichMecha].update();
              rojarArm[!whichMecha].setHeight(1600);
              rojarArm[!whichMecha].update();
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 10:
              whichMecha = !whichMecha; //途中で機構入れ替え！！
              wayPointSignature++;
              break;
            case 11:
              static int armPhaseRight = 1, hangerHasDoneFlagRight = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhaseRight == 1)   //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1020); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlagRight)
                {
                  hangerHasDoneFlagRight = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                  rojarArm[whichMecha].setHeight(1270); //洗濯物離すために大幅に下げる
                }
              }
              if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats() && hangerHasDoneFlagRight)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holder[whichMecha].release(whichServo);
                  seq = 2;
                }
                if (800 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp(whichServo);
                  seq = 3;
                }
                if (1600 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhaseRight = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  rojarArm[whichMecha].setHeight(1500);
                }
              }
              if (armPhaseRight == 2)
              {
                if (hanger[whichMecha].stats() && rojarArm[whichMecha].stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    clothHangerTimer.stop();
                    clothHangerTimer.reset();
                    robotLocation.sendNext(); //9
                    accelAlgorithm.setPositionChangedFlag();
                    rojarArm[whichMecha].setHeight(1910);
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 12:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 13:
              OmniKinematics.setMaxPWM(0.15);
              robotLocation.sendNext(); //横移動 10
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 14:
              static bool initialFlagSec = 1, releasedFlagSec = 0;
              if (initialFlagSec)
              {
                OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
                clothHangerTimer.start();
                initialFlagSec = 0;
              }
              if (clothHangerTimer.read_ms() > 300)
              {
                robotLocation.sendNext(); //11
                accelAlgorithm.setPositionChangedFlag();
                rojarArm[whichMecha].setMaxPWM(0.35);
                rojarArm[whichMecha].setHeight(1530);
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlagSec = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlagSec)
              {
                holder[whichMecha].half(!whichServo);
                wayPointSignature++;
                clothHangerTimer.start();
              }
              break;
            case 15:
              if (650 < clothHangerTimer.read_ms())
              {
                robotLocation.sendNext(); //12
                accelAlgorithm.setPositionChangedFlag();
                wayPointSignature++;
                clothHangerTimer.stop();
                clothHangerTimer.reset();
              }
              break;
            case 16:
              rojarArm[whichMecha].setHeight(0);
              rojarArm[whichMecha].setMaxPWM(Robot.estimateRojarArmMaxPWM);
              robotLocation.sendNext(); //13
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 17:
              holder[right].grasp(!whichServo);
              holder[right].grasp(whichServo);
              holder[left].grasp(!whichServo);
              holder[left].grasp(whichServo);
              pegAttacher[right].reload();
              pegAttacher[left].reload();
              robotLocation.sendNext(); //14
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 18:
              holder[right].free(!whichServo);
              holder[right].free(whichServo);
              holder[left].free(!whichServo);
              holder[left].free(whichServo);
              while (1)
              {
                allUpdate();
              }
              break;
            default:
              while (1)
                ;
          }
        }
        break;
      case RED_BACK_PRE_Sheets:
      case RED_BACK_FINAL_Sheets:
      case BLUE_BACK_PRE_Sheets:
      case BLUE_BACK_FINAL_Sheets:
        updateLimitSwitchBar();
        if (getLimitSwitchBarStats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(590));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(590), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
            OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
            wayPointSignature++;
          }
          flag++;
        }

        limitSwitchSide[whichMecha].update();
        if (limitSwitchSide[whichMecha].stats() && wayPointSignature == 5)
        {
          accelAlgorithm.setCurrentXPosition(9 < currentRunningMode ? -375 : 375);
          robotLocation.setCurrentPoint(9 < currentRunningMode ? -375 : 375, robotLocation.getYLocationData(), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            wayPointSignature++;
          }
          flag++;
        }

        if (robotLocation.checkMovingStats(accelAlgorithm.getStats()))
        {
          switch (wayPointSignature)
          {
            case 1:
              accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
              robotLocation.sendNext(); //三本目のポール少し手前の位置
              accelAlgorithm.setPositionChangedFlag();
              OmniKinematics.setMaxPWM(0.3);
              rojarArm[whichMecha].setHeight(2060); //2100-洗濯物干し最適高さ , 2850-洗濯バサミ最適高さ
              wayPointSignature++;
              break;

            case 2:
              updateLimitSwitchBar();
              if (!getLimitSwitchBarStats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 20, robotLocation.getYawStatsData());
              }
              break;

            case 3:
              static int armPhase = 1, hangerHasDoneFlag = 0;    //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhase == 1) //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1020); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                }
              }
              static int initialChangeHeightFlag = 1;
              if (hanger[whichMecha].stats() && hangerHasDoneFlag && initialChangeHeightFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1300 && seq == 1)
                {
                  holder[whichMecha].release(whichServo);
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp(whichServo);
                  rojarArm[whichMecha].setHeight(2500);
                  rojarArm[whichMecha].update();
                  seq = 3;
                }
                if (3000 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  initialChangeHeightFlag = 0;
                }
              }
              if (rojarArm[whichMecha].stats() && armPhase == 2)
              {
                if (hanger[whichMecha].stats() && hangerHasDoneFlag) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 370 && seq == 1) //`hutatume 1700ms
                  {
                    pegAttacher[whichMecha].launch();
                    seq = 2;
                  }
                  if (370 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    pegAttacher[whichMecha].reload();
                    seq = 3;
                  }
                  if (700 < clothHangerTimer.read_ms() && seq == 3)
                  {
                    robotLocation.sendNext(); //次の座標を送信
                    accelAlgorithm.setPositionChangedFlag();
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 4:
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;

            case 5:
              limitSwitchSide[whichMecha].update();
              if (!limitSwitchSide[whichMecha].stats())
              {
                OmniKinematics.setMaxPWM(0.13);
                robotLocation.setCurrentPoint(9 < currentRunningMode ? (robotLocation.getXLocationData() - 10) : (robotLocation.getXLocationData() + 10), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
                break;
              }
              break;

            case 6:
              static bool initialFlag = 1;
              if (initialFlag)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
                OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
                rojarArm[whichMecha].setMaxPWM(0.4);
                rojarArm[whichMecha].setHeight(1930);
                rojarArm[whichMecha].update();
                initialFlag = 0;
              }
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;

            case 7:
              holder[whichMecha].release(!whichServo);
              rojarArm[whichMecha].setMaxPWM(Robot.estimateRojarArmMaxPWM);
              rojarArm[whichMecha].setHeight(0);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;

            case 8:
              holder[whichMecha].grasp(whichServo);
              holder[whichMecha].grasp(!whichServo);
              pegAttacher[whichMecha].reload();
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;

            case 9:
              holder[whichMecha].free(whichServo);
              holder[whichMecha].free(!whichServo);
              holder[!whichMecha].free(whichServo);
              holder[!whichMecha].free(!whichServo);
              while (1)
              {
                allUpdate();
              }
              break;

            default:
              while (1)
              {
              }
          }
        }
    }
  }

#endif //GAME_MODECHANGE_ONBOARDSWITCH

#ifdef BUDEGGER
  Timer printTimer;
  printTimer.start();
  IMU.setup();
  STLinkTerminal.printf("\033[%d;%dH", 1, 0);
  STLinkTerminal.printf("\033[2K");
  STLinkTerminal.printf("Encoder\033[3CencoderXAxis\t\tencoderYAxis\t\trightRojarEncoder\tleftRojarEncoder\trightLauncherEncoder\tleftLauncherEncoder");
  STLinkTerminal.printf("\033[%d;%dH", 4, 0);
  STLinkTerminal.printf("\033[2K");
  STLinkTerminal.printf("LimitSW\033[3CrightRojarSwitch\tleftRojarSwitch\t\tforwardRightDriveSwitch\tforwardleftDriveSwitch\trightDriveSwitch\tleftDriveSwitch");
  while (1)
  {
    for (int i = 0; i < 2; i++)
    {
      rojarArm[i].setEncoderPulse(rojarArmEncoder[i].getPulses());
      rojarArm[i].update();
      pegAttacher[i].update();
      hanger[i].setEncoderPulse(clothHangEncoder[i].getPulses());
      hanger[i].update();
      limitSwitchBar[i].update();
      limitSwitchRojarArm[i].update();
      limitSwitchSide[i].update();
    }
    //accelAlgorithm.update();
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
    static unsigned long int prevPrint;
    if ((printTimer.read_us() - prevPrint) > 1000)
    {
      STLinkTerminal.printf("\033[%d;%dH", 2, 11);
      STLinkTerminal.printf("\033[2K%d\t\t\t%d\t\t\t%d\t\t\t%d\t\t\t%d\t\t\t%d", encoderXAxis.getPulses(), encoderYAxis.getPulses(), rojarArmEncoder[right].getPulses(), rojarArmEncoder[left].getPulses(), clothHangEncoder[right].getPulses(), clothHangEncoder[left].getPulses());
      STLinkTerminal.printf("\033[%d;%dH", 5, 11);
      STLinkTerminal.printf("\033[2K%d\t\t\t%d\t\t\t%d\t\t\t%d\t\t\t%d\t\t\t%d", limitSwitchRojarArm[right].stats(), limitSwitchRojarArm[left].stats(), limitSwitchBar[right].stats(), getLimitSwitchBarStats(), limitSwitchSide[right].stats(), limitSwitchSide[left].stats());
      STLinkTerminal.printf("\033[7;0H\033[2KINTERVAL TIME : %ld[us]", (long int)(printTimer.read_us() - prevPrint));
      prevPrint = printTimer.read_us();
    }
  }
#endif //BUDEGGER

#ifdef CONTROLPANEL
  union locationData {
    float currentPosition; //4 bytes
    uint8_t byteToSend[sizeof(float)];
  } xAxis, yAxis, yawAxis;
  enum
  {
    rojarRight,
    rojarLeft,
    hangRight,
    hangLeft,
  } EncoderSets;
  struct
  {
    int currentPulses;
  } EncoderPulses[sizeof(EncoderSets)];
  //IMU.setup();
  while (1)
  {
    //accelAlgorithm.update();
    //accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    xAxis.currentPosition = 120.321; //(float)accelAlgorithm.getCurrentXPosition();
    yAxis.currentPosition = 235.532; //(float)accelAlgorithm.getCurrentYPosition();
    yawAxis.currentPosition = 0.643; //(float)accelAlgorithm.getCurrentYawPosition();
    EncoderPulses[rojarRight].currentPulses = rojarArmEncoder[right].getPulses() + 200;
    EncoderPulses[rojarLeft].currentPulses = rojarArmEncoder[left].getPulses();
    EncoderPulses[hangRight].currentPulses = clothHangEncoder[right].getPulses();
    EncoderPulses[hangLeft].currentPulses = clothHangEncoder[left].getPulses();
    int Packet[1 + sizeof(float) * 3 + 8]; // 位置x3 x4 12byte float x3  エンコーダx4 x2 8byte int16_t x4
    Packet[0] = 'd';
    for (int i = 0; i < sizeof(float); i++)
    {
      Packet[i + 1] = xAxis.byteToSend[i];
      Packet[sizeof(float) + i + 1] = yAxis.byteToSend[i];
      Packet[(sizeof(float) * 2) + i + 1] = yawAxis.byteToSend[i];
    }
    Packet[13] = (uint8_t)(EncoderPulses[rojarRight].currentPulses & 0xff);
    Packet[14] = (uint8_t)((EncoderPulses[rojarRight].currentPulses >> 8) & 0xff);
    Packet[15] = 0; //(EncoderPulses[rojarLeft].currentPulses >> 8) & 0xff;
    Packet[16] = 0; //EncoderPulses[rojarLeft].currentPulses & 0xff;
    Packet[17] = 0; //(EncoderPulses[hangRight].currentPulses >> 8) & 0xff;
    Packet[18] = 0; //EncoderPulses[hangRight].currentPulses & 0xff;
    Packet[19] = 0; //(EncoderPulses[hangLeft].currentPulses >> 8) & 0xff;
    Packet[20] = 0; //EncoderPulses[hangLeft].currentPulses & 0xff;
    for (int i = 0; i < 21; i++)
    {
      STLinkTerminal.printf("%x\t", Packet[i]);
    }
    STLinkTerminal.printf("\r\n");
    union {
      float currentPosition; //4 bytes
      uint8_t byteToSend[sizeof(float)];
    } RxAxis, RyAxis, RyawAxis;
    for (int i = 0; i < 4; i++)
    {
      RxAxis.byteToSend[i] = Packet[1 + i];
      RyAxis.byteToSend[i] = Packet[1 + 4 + i];
      RyawAxis.byteToSend[i] = Packet[1 + 8 + i];
    }
    STLinkTerminal.printf("%f\t%f\t%f", RxAxis.currentPosition, RyAxis.currentPosition, RyawAxis.currentPosition);

    STLinkTerminal.printf("\r\n");

    UIF.transmit(21, (int *)Packet);
  }

#endif //CONTROLPANEL

#ifdef CONTROLPANEL_GETMODE_TEST
  while (1)
  {
    static uint8_t data[1], state;
    state = UIF.receive(data);
    STLinkTerminal.printf("%d, %d\r\n", state, data[0]);
  }

#endif //CONTROLPANEL_GETMODE_TEST

#ifdef TEST_DRIVE
  robotLocation.addPoint(0, -10, 0);
  robotLocation.addPoint(10, -10, 0);
  robotLocation.addPoint(10, -0, 0);
  robotLocation.addPoint(0, 0, 0);
  IMU.setup();
  while (1) //モードセレクト処理
  {
    startButton.update();
    if (startButton.stats())
      break;
  }
  while (1)
  {
    static unsigned int wayPointSignature = 0;
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    accelAlgorithm.update();
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
    int MDD1Packet[6] = {
        driverPWMOutput[0] < 0 ? 0 : (int)(driverPWMOutput[0] * 100),
        driverPWMOutput[0] > 0 ? 0 : -(int)(driverPWMOutput[0] * 100),
        driverPWMOutput[1] < 0 ? 0 : (int)(driverPWMOutput[1] * 100),
        driverPWMOutput[1] > 0 ? 0 : -(int)(driverPWMOutput[1] * 100),
        driverPWMOutput[2] < 0 ? 0 : (int)(driverPWMOutput[2] * 100),
        driverPWMOutput[2] > 0 ? 0 : -(int)(driverPWMOutput[2] * 100),
    };
    MDDSlave[0].transmit(6, MDD1Packet);
    if (robotLocation.checkMovingStats(accelAlgorithm.getStats()))
    {
      if (wayPointSignature == 4)
        break;
      robotLocation.sendNext();
      wayPointSignature++;
    }
    STLinkTerminal.printf("%4.4lf,%4.4lf,%4.4lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
  }
  while (1)
  {
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    accelAlgorithm.update();
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
    int MDD1Packet[6] = {
        driverPWMOutput[0] < 0 ? 0 : (int)(driverPWMOutput[0] * 100),
        driverPWMOutput[0] > 0 ? 0 : -(int)(driverPWMOutput[0] * 100),
        driverPWMOutput[1] < 0 ? 0 : (int)(driverPWMOutput[1] * 100),
        driverPWMOutput[1] > 0 ? 0 : -(int)(driverPWMOutput[1] * 100),
        driverPWMOutput[2] < 0 ? 0 : (int)(driverPWMOutput[2] * 100),
        driverPWMOutput[2] > 0 ? 0 : -(int)(driverPWMOutput[2] * 100),
    };
    MDDSlave[0].transmit(6, MDD1Packet);
    STLinkTerminal.printf("%4.4lf,%4.4lf,%4.4lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
  }
#endif //TEST_DRIVE

#ifdef TEST_RojarArm_UpDownLoop
  while (1)
  {
    startButton.update();
    if (startButton.stats())
      break;
  }
  while (1)
  {
    rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
    rojarArmRight.update();
    static int armPhase = 0;
    if (rojarArmRight.stats() && armPhase == 0)
    {
      wait(2);
      armPhase++;
      rojarArmRight.setHeight(3200);
      rojarArmRight.update();
    }
    else if (rojarArmRight.stats() && armPhase == 1)
    {
      wait(2);
      armPhase = 0;
      rojarArmRight.setHeight(0);
      rojarArmRight.update();
    }
  }
#endif //TEST_RojarArm_UpDownLoop

#ifdef TEST_PEGLaunch
  DebounceSwitch startButton(USER_BUTTON, 'd');
  while (1)
  {
    startButton.update();
    if (startButton.stats() == 1)
      break;
  }
  Peg pegAttacher(PE_12, PE_14, 0.5, 0.5);
  pegAttacher.launch();
  while (1)
  {
    pegAttacher.update();
  }
#endif // TEST_PEGLaunch

#ifdef TEST_HangerMovement
  TimerForLCD.start();
  while (1)
  {
    startButton.update();
    if (startButton.stats() == 1)
      break;
  }
  hanger[0].setMaxPWM(0.5);
  hanger[0].setLength(1200);
  hanger[1].setMaxPWM(0.5);
  hanger[1].setLength(1200);
  while (1)
  {
    static unsigned long int prevDisplayed = 0;
    if (((TimerForLCD.read_ms() - prevDisplayed) > 100)) //about 10Hz flash rate
    {
      LCDDriver.clear();
      LCDDriver.home();
      serialLCD.printf("pul:%d %d", clothHangEncoder[0].getPulses(), clothHangEncoder[1].getPulses());
      prevDisplayed = TimerForLCD.read_ms();
    }
    hanger[0].setEncoderPulse(clothHangEncoder[0].getPulses());
    hanger[0].update();
    hanger[1].setEncoderPulse(clothHangEncoder[1].getPulses());
    hanger[1].update();
    int MDDMeca1Packet[4] = {
        0,
        0,
        (int)(hangerPWM[0][0] * 100),
        (int)(hangerPWM[0][1] * 100),
    };
    int MDDMeca2Packet[4] = {
        0,
        0,
        (int)(hangerPWM[1][0] * 100),
        (int)(hangerPWM[1][1] * 100),
    };
    MDDSlave[Meca1].transmit(4, MDDMeca1Packet);
    MDDSlave[Meca2].transmit(4, MDDMeca2Packet);
    if (hanger[0].stats() && hanger[1].stats())
      break;
  }
  hanger[0].setLength(0);
  hanger[1].setLength(0);
  while (1)
  {
    static unsigned long int prevDisplayed = 0;
    if (((TimerForLCD.read_ms() - prevDisplayed) > 100)) //about 10Hz flash rate
    {
      LCDDriver.clear();
      LCDDriver.home();
      serialLCD.printf("pul:%d %d", clothHangEncoder[0].getPulses(), clothHangEncoder[1].getPulses());
      prevDisplayed = TimerForLCD.read_ms();
    }
    hanger[0].setEncoderPulse(clothHangEncoder[0].getPulses());
    hanger[0].update();
    hanger[1].setEncoderPulse(clothHangEncoder[1].getPulses());
    hanger[1].update();
    int MDDMeca1Packet[4] = {
        0,
        0,
        (int)(hangerPWM[0][0] * 100),
        (int)(hangerPWM[0][1] * 100),
    };
    int MDDMeca2Packet[4] = {
        0,
        0,
        (int)(hangerPWM[1][0] * 100),
        (int)(hangerPWM[1][1] * 100),
    };
    MDDSlave[Meca1].transmit(4, MDDMeca1Packet);
    MDDSlave[Meca2].transmit(4, MDDMeca2Packet);
  }
#endif //TEST_HangerMovement

#ifdef TEST_HoldServo
  while (0)
  {
    startButton.update();
    if (startButton.stats() == 1)
      break;
  }
  while (1)
  {
    holder[0].grasp(right);
    holder[0].grasp(left);
    holder[1].release(right);
    holder[1].release(left);
    wait(2.5);
    holder[0].release(right);
    holder[0].release(left);
    holder[1].grasp(right);
    holder[1].grasp(left);

    wait(2.5);
  }
#endif //TEST_HoldServo

#ifdef TEST_RojarArmUpDownOnce
  while (1)
  {
    startButton.update();
    if (startButton.stats() == 1)
      break;
  }
  TimerForLCD.start();
  rojarArm[right].setMaxPWM(0.5);
  rojarArm[right].setHeight(2400);
  rojarArm[left].setMaxPWM(0.5);
  rojarArm[left].setHeight(3200);
  while (1)
  {
    static unsigned long int prevDisplayed = 0;
    if (((TimerForLCD.read_ms() - prevDisplayed) > 100)) //about 10Hz flash rate
    {
      LCDDriver.clear();
      LCDDriver.home();
      serialLCD.printf("pul:%d %d", rojarArmEncoder[0].getPulses(), rojarArmEncoder[1].getPulses());
      prevDisplayed = TimerForLCD.read_ms();
    }
    STLinkTerminal.printf("PULSE:%d \tARM STATS(extend):%d\r\n", rojarArmEncoder[right].getPulses(), rojarArm[right].stats());
    rojarArm[right].setEncoderPulse(rojarArmEncoder[right].getPulses());
    rojarArm[right].update();
    rojarArm[left].setEncoderPulse(rojarArmEncoder[left].getPulses());
    rojarArm[left].update();
    int MDDMeca1Packet[2] = {
        (int)(rojarArmPWM[0][1] * 100),
        (int)(rojarArmPWM[0][0] * 100),
    };
    int MDDMeca2Packet[2] = {
        (int)(rojarArmPWM[1][1] * 100),
        (int)(rojarArmPWM[1][0] * 100),
    };
    MDDSlave[Meca1].transmit(2, MDDMeca1Packet);
    MDDSlave[Meca2].transmit(2, MDDMeca2Packet);

    if (rojarArm[left].stats())
      break;
  }
  wait(2);
  rojarArm[right].setHeight(0);
  rojarArm[left].setHeight(0);
  while (1)
  {
    static unsigned long int prevDisplayed = 0;
    if (((TimerForLCD.read_ms() - prevDisplayed) > 100)) //about 10Hz flash rate
    {
      LCDDriver.clear();
      LCDDriver.home();
      serialLCD.printf("pul:%d %d", rojarArmEncoder[0].getPulses(), rojarArmEncoder[1].getPulses());
      prevDisplayed = TimerForLCD.read_ms();
    }
    STLinkTerminal.printf("PULSE:%d \tARM STATS(reduce):%d\r\n", rojarArmEncoder[right].getPulses(), rojarArm[right].stats());
    rojarArm[right].setEncoderPulse(rojarArmEncoder[right].getPulses());
    rojarArm[right].update();
    rojarArm[left].setEncoderPulse(rojarArmEncoder[left].getPulses());
    rojarArm[left].update();
    int MDDMeca1Packet[2] = {
        (int)(rojarArmPWM[0][1] * 100),
        (int)(rojarArmPWM[0][0] * 100),
    };
    int MDDMeca2Packet[4] = {
        (int)(rojarArmPWM[1][1] * 100),
        (int)(rojarArmPWM[1][0] * 100),
    };
    MDDSlave[Meca1].transmit(2, MDDMeca1Packet);
    MDDSlave[Meca2].transmit(2, MDDMeca2Packet);
  }
#endif //TEST_RojarArmUpDownOnce
}

void allUpdate()
{
  for (int i = 0; i < 2; i++)
  {
    rojarArm[i].setEncoderPulse(rojarArmEncoder[i].getPulses());
    rojarArm[i].update();
    pegAttacher[i].update();
    hanger[i].setEncoderPulse(clothHangEncoder[i].getPulses());
    hanger[i].update();
  }
  accelAlgorithm.update();
  accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
  OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
  uint8_t MDDDrivePacket[6] = {
      (uint8_t)driverPWMOutput[0] < 0 ? 0 : (uint8_t)(driverPWMOutput[0] * 100),
      (uint8_t)driverPWMOutput[0] > 0 ? 0 : (uint8_t)(-driverPWMOutput[0] * 100),
      (uint8_t)driverPWMOutput[1] < 0 ? 0 : (uint8_t)(driverPWMOutput[1] * 100),
      (uint8_t)driverPWMOutput[1] > 0 ? 0 : (uint8_t)(-driverPWMOutput[1] * 100),
      (uint8_t)driverPWMOutput[2] < 0 ? 0 : (uint8_t)(driverPWMOutput[2] * 100),
      (uint8_t)driverPWMOutput[2] > 0 ? 0 : (uint8_t)(-driverPWMOutput[2] * 100),
  };
  uint8_t MDDMeca1Packet[6] = {
      (uint8_t)(rojarArmPWM[0][1] * 100),
      (uint8_t)(rojarArmPWM[0][0] * 100),
      (uint8_t)(hangerPWM[0][1] * 100),
      (uint8_t)(hangerPWM[0][0] * 100),
      (uint8_t)(pegAttacherPWM[0][1] * 100),
      (uint8_t)(pegAttacherPWM[0][0] * 100),
  };
  uint8_t MDDMeca2Packet[6] = {
      (uint8_t)(rojarArmPWM[1][0] * 100),
      (uint8_t)(rojarArmPWM[1][1] * 100),
      (uint8_t)(hangerPWM[1][1] * 100),
      (uint8_t)(hangerPWM[1][0] * 100),
      (uint8_t)(pegAttacherPWM[1][1] * 100),
      (uint8_t)(pegAttacherPWM[1][0] * 100),
  };
  MDDSlave[Drive].transmit(6, MDDDrivePacket);
  MDDSlave[Meca1].transmit(6, MDDMeca1Packet);
  MDDSlave[Meca2].transmit(6, MDDMeca2Packet);
}
bool getRobotModeWithSwitch()
{
  static bool stateHasChanged, previousState;
  static unsigned int pushedCounter;
  onBoardSwitch.update();
  startButton.update();
  if (onBoardSwitch.stats() != previousState)
  {
    stateHasChanged = 1;
  }
  else
    stateHasChanged = 0;
  if (onBoardSwitch.stats() != previousState)
  {
    stateHasChanged = 1;
  }
  else
    stateHasChanged = 0;
  if (onBoardSwitch.stats() && stateHasChanged)
  {
    pushedCounter++;
    currentRunningMode = pushedCounter % 8;
  }
  previousState = onBoardSwitch.stats();
  switch (currentRunningMode)
  {
    case 0:
      modeIndicatorLED1 = 0;
      modeIndicatorLED2 = 0;
      modeIndicatorLED3 = 0;
      break;
    case 1:
      modeIndicatorLED1 = 1;
      modeIndicatorLED2 = 0;
      modeIndicatorLED3 = 0;
      break;
    case 2:
      modeIndicatorLED1 = 0;
      modeIndicatorLED2 = 1;
      modeIndicatorLED3 = 0;
      break;
    case 3:
      modeIndicatorLED1 = 1;
      modeIndicatorLED2 = 1;
      modeIndicatorLED3 = 0;
      break;
    case 4:
      modeIndicatorLED1 = 0;
      modeIndicatorLED2 = 0;
      modeIndicatorLED3 = 1;
      break;
    case 5:
      modeIndicatorLED1 = 1;
      modeIndicatorLED2 = 0;
      modeIndicatorLED3 = 1;
      break;
    case 6:
      modeIndicatorLED1 = 0;
      modeIndicatorLED2 = 1;
      modeIndicatorLED3 = 1;
      break;
    case 7:
      modeIndicatorLED1 = 1;
      modeIndicatorLED2 = 1;
      modeIndicatorLED3 = 1;
      break;
  }
  if (startButton.stats())
    return 1;
  return 0;
}

void updateLimitSwitchBar()
{
  limitSwitchBar[right].update();
  limitSwitchBar[left].update();
}

bool getLimitSwitchBarStats()
{
  if (limitSwitchBar[right].stats() || limitSwitchBar[left].stats())
    return 1;
  return 0;
}

void driveAutoConverger()
{
  static double outMax = 5, outMin = -5;
  static double pGain = 0.1, iGain = 0, dGain = 0; //各制御のゲイン
  static double pTerm[2], iTerm[2], dTerm[2];      //各タームの制御量
  static unsigned long loopTime;
  pidLoopTimer.start();
  limitSwitchBar[right].disableStateUpdate();
  limitSwitchBar[left].disableStateUpdate();
  limitSwitchSide[right].disableStateUpdate();
  limitSwitchBar[right].setButtonState(1);
  limitSwitchBar[left].setButtonState(1);
  limitSwitchSide[right].setButtonState(1);
  if ((pidLoopTimer.read_ms() - loopTime) > 30)
  {
    double error[2];
    error[0] = 0;
    error[1] = 0;
    double pidOut[2];
    double currentXLocation = robotLocation.getXLocationData();
    double currentYLocation = robotLocation.getYLocationData();
    static double prevXLocation;
    static double prevYLocation;
    error[0] = currentXLocation - accelAlgorithm.getCurrentXPosition();
    error[1] = currentYLocation - accelAlgorithm.getCurrentYPosition();
    dTerm[0] = (currentXLocation - prevXLocation) * dGain;
    dTerm[1] = (currentYLocation - prevYLocation) * dGain;
    for (int i = 0; i < 2; i++)
    {
      pTerm[i] = error[i] * pGain;  //偏差に応じた制御量決定
      iTerm[i] += error[i] * iGain; //各軸の偏差を蓄積させ制御量を決定
      iTerm[i] = iTerm[i] > outMax ? outMax : iTerm[i];
      iTerm[i] = iTerm[i] < outMin ? outMin : iTerm[i];
      pidOut[i] = pTerm[i] + iTerm[i] - dTerm[i];
      pidOut[i] = pidOut[i] > outMax ? outMax : pidOut[i];
      pidOut[i] = pidOut[i] < outMin ? outMin : pidOut[i];
    }
    accelAlgorithm.setCurrentXPosition(accelAlgorithm.getCurrentXPosition() + pidOut[0]);
    accelAlgorithm.setCurrentYPosition(accelAlgorithm.getCurrentYPosition() + pidOut[1]);
    prevXLocation = currentXLocation;
    prevYLocation = currentYLocation;
    loopTime = pidLoopTimer.read_ms();
  }
}