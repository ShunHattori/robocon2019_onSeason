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
#include "NewHavenDisplay.h"
#include "SensorSource\DebounceSwitch.h"
#include "SensorSource\MPU9250.h"
#include "SensorSource\QEI.h"
#include "UnitProtocol.hpp"
#include "mbed.h"

//#define GAME_MODECHANGE_ONBOARDSWITCH //基板上の青スイッチで動作切り替え（シーツ・バスタオル）
//#define TEST_DRIVE
//#define TEST_RojarArm_UpDownLoop
//#define TEST_PEGLaunch      //tested on 6/20(Thur)
//#define TEST_HangerMovement //tested on 6/20(Thur)
#define TEST_HoldServo
//#define TEST_RojarArmUpDownOnce

enum gameMode
{ //剰余計算のために作成中は順番を崩している
  RED_FRONT_PRE_bathTowelLeft,
  RED_MIDDLE_PRE_bathTowelRight,
  RED_MIDDLE_PRE_bathTowelLeft,
  RED_MIDDLE_PRE_bathTowelboth,
  RED_BACK_Sheets,
  RED_MIDDLE_FINAL_bathTowelRight,
  RED_MIDDLE_FINAL_bathTowelLeft,
  RED_MIDDLE_FINAL_bathTowelboth,

  //↓ not yet
  BLUE_FRONT_PRE_bathTowelRight,
  BLUE_MIDDLE_PRE_bathTowelLeft,
  BLUE_MIDDLE_PRE_bathTowelRight,
  BLUE_MIDDLE_PRE_bathTowelboth,
  BLUE_MIDDLE_FINAL_bathTowelLeft,
  BLUE_MIDDLE_FINAL_bathTowelRight,
  BLUE_MIDDLE_FINAL_bathTowelboth,
  BLUE_BACK_Sheets,
};

struct baudRate
{
  const long HardwareSerial = 256000;
  const int LCD = 9600;
  const long I2C = 400000;
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
  const double estimatePegMaxPWM = 0.6;
  const double estimateHangerMaxPWM = 0.6;
  const double estimateRojarArmMaxPWM = 0.75;
  const double PegVoltageImpressionTime = 0.35;
} Robot;

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
  //PinName LCD1TX = PB_6;
  //PinName LCD1RX = PB_15;
  PinName UIFTX = PB_6; //user interface
  PinName UIFRX = PB_15;
  PinName MDD1TX = PD_5;
  PinName MDD1RX = PD_6;
  PinName MDD2TX = PC_10;
  PinName MDD2RX = PC_11;
  PinName MDD3TX = PC_6;
  PinName MDD3RX = PC_7;
} serialDevice;

struct
{
  PinName toBegin = PG_2;
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
  PinName clothHangRightEncoderAPulse = PB_1;
  PinName clothHangRightEncoderBPulse = PC_2;
  PinName clothHangLeftEncoderAPulse = PD_3;
  PinName clothHangLeftEncoderBPulse = PD_4;
  PinName rojarArmRightEncoderAPulse = PB_8;
  PinName rojarArmRightEncoderBPulse = PB_9;
  PinName rojarArmLeftEncoderAPulse = PA_5;
  PinName rojarArmLeftEncoderBPulse = PA_6;
  PinName holderRightServoR = PF_8; //pf8
  PinName holderRightServoL = PF_7; //pf7
  PinName holderLeftServoR = PE_6;
  PinName holderLeftServoL = PE_5;
} MecaPin;

double driverPWMOutput[3];
static double pegAttacherPWM[2][2]; //right(CW,CCW),left(CW,CCW)
static double hangerPWM[2][2];      //right(CW,CCW),left(CW,CCW)
static double rojarArmPWM[2][2];    //right(CW,CCW),left(CW,CCW)

Serial STLinkTerminal(USBTX, USBRX, SerialBaud.HardwareSerial); //Surfaceのターミナルとの通信用ポート
//Serial serialLCD(serialDevice.LCD1TX, serialDevice.LCD1RX, SerialBaud.LCD);
//NewHavenDisplay LCDDriver(serialLCD);
Timer TimerForQEI;                  //エンコーダクラス用共有タイマー
Timer TimerForLCD;                  //LCD更新用タイマー
Timer clothHangerTimer;             //機構シーケンス用タイマー
DigitalOut modeIndicatorLED1(LED1); //一時的モード切替表示用LED
DigitalOut modeIndicatorLED2(LED2); //一時的モード切替表示用LED
DigitalOut modeIndicatorLED3(LED3); //一時的モード切替表示用LED
MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, SerialBaud.I2C);
UnitProtocol UIF(serialDevice.UIFTX, serialDevice.UIFRX, SerialBaud.HardwareSerial);
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

int getRunningMode();
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
  int MDDDrivePacket[6] = {
      driverPWMOutput[0] < 0 ? 0 : (int)(driverPWMOutput[0] * 100),
      driverPWMOutput[0] > 0 ? 0 : -(int)(driverPWMOutput[0] * 100),
      driverPWMOutput[1] < 0 ? 0 : (int)(driverPWMOutput[1] * 100),
      driverPWMOutput[1] > 0 ? 0 : -(int)(driverPWMOutput[1] * 100),
      driverPWMOutput[2] < 0 ? 0 : (int)(driverPWMOutput[2] * 100),
      driverPWMOutput[2] > 0 ? 0 : -(int)(driverPWMOutput[2] * 100),
  };
  int MDDMeca1Packet[6] = {
      (int)(rojarArmPWM[0][1] * 100),
      (int)(rojarArmPWM[0][0] * 100),
      (int)(hangerPWM[0][0] * 100),
      (int)(hangerPWM[0][1] * 100),
      (int)(pegAttacherPWM[0][1] * 100),
      (int)(pegAttacherPWM[0][0] * 100),
  };
  int MDDMeca2Packet[6] = {
      (int)(rojarArmPWM[1][1] * 100),
      (int)(rojarArmPWM[1][0] * 100),
      (int)(hangerPWM[1][0] * 100),
      (int)(hangerPWM[1][1] * 100),
      (int)(pegAttacherPWM[1][1] * 100),
      (int)(pegAttacherPWM[1][0] * 100),
  };
  int UIFDataPacket[54] = {

  };
  MDDSlave[Drive].transmit(6, MDDDrivePacket);
  MDDSlave[Meca1].transmit(6, MDDMeca1Packet);
  MDDSlave[Meca2].transmit(6, MDDMeca2Packet);
  UIF.transmit(54, UIFDataPacket);
}

int main(void)
{
  for (int i = 0; i < 2; i++)
  {
    hanger[i].setMaxPWM(Robot.estimateHangerMaxPWM);
    rojarArm[i].setMaxPWM(Robot.estimateRojarArmMaxPWM);
    holder[i].free('r');
    holder[i].free('l');
  }
  accelAlgorithm.setMaxOutput(Robot.estimateDriveMaxPWM);
  accelAlgorithm.setMinOutput(Robot.estimateDriveMinPWM);
  OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
#ifdef GAME_MODECHANGE_ONBOARDSWITCH

  IMU.setup();
  //TimerForLCD.start();
  //serialLCD.printf("WAITING...");

  static int currentRunningMode, whichMecha;
  while (1) //ボタン待機
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
      break;

    //currentRunningMode = getRunningMode();
  }

  switch (currentRunningMode)
  {
    case RED_FRONT_PRE_bathTowelLeft: //左ロジャー
      whichMecha = left;
      robotLocation.addPoint((123), -(205)); //近づいてリミット監視開始 初期位置ずれてるから長めになってる！
      robotLocation.addPoint((207), -(190)); //竿目印のところまで移動(ちょっと後ろ目)
      robotLocation.addPoint((207), -(190)); //ハサミつく位置まで前に
      robotLocation.addPoint((207), -(190)); //最初に直進 //198
      robotLocation.addPoint((307), -(200)); //洗濯物横に引っ張る //202
      robotLocation.addPoint((307), -(190)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((307), -(170));
      robotLocation.addPoint(30, 30); //初期位置
      break;
    case RED_MIDDLE_PRE_bathTowelLeft: //(横掛け), 左ロジャー
      whichMecha = left;
      robotLocation.addPoint(10, -(390));    //二本目のポール前
      robotLocation.addPoint((123), -(395)); //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((208), -(380)); //竿目印のところまで移動
      robotLocation.addPoint((208), -(390)); //ハサミつく位置まで前に
      robotLocation.addPoint((208), -(390)); //最初に直進
      robotLocation.addPoint((308), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint((308), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((308), -(370));
      robotLocation.addPoint(30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);     //初期位置
      break;
    case RED_MIDDLE_PRE_bathTowelRight: //縦:112-170,横112-205 (縦掛け), 右ロジャー
      whichMecha = right;
      robotLocation.addPoint(10, -(390));    //二本目のポール前
      robotLocation.addPoint((123), -(395)); //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((320), -(380)); //竿目印のところまで移動
      robotLocation.addPoint((320), -(390)); //ハサミつく位置まで前に
      robotLocation.addPoint((320), -(390)); //最初に直進
      robotLocation.addPoint((360), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint((360), -(385)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((360), -(370));
      robotLocation.addPoint(30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);     //初期位置
      break;
    case RED_MIDDLE_PRE_bathTowelboth:       //予選(左ロジャー縦掛け),(右ロジャー横掛け)
      whichMecha = left;                     //途中で変わる!!
      robotLocation.addPoint(10, -(390));    //1二本目のポール前
      robotLocation.addPoint((123), -(395)); //2リミット接触前ちょこちょこ進む
      robotLocation.addPoint((208), -(390)); //3竿目印のところまで移動
      robotLocation.addPoint((208), -(390)); //4ハサミつく位置まで前に
      robotLocation.addPoint((208), -(390)); //5最初に直進
      robotLocation.addPoint((308), -(400)); //6洗濯物横に引っ張る
      robotLocation.addPoint((308), -(390)); //7ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((320), -(390)); //8竿目印のところまで移動     2つ目！
      robotLocation.addPoint((320), -(390)); //9ハサミつく位置まで前に
      robotLocation.addPoint((360), -(400)); //10洗濯物横に引っ張る
      robotLocation.addPoint((360), -(385)); //11ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((360), -(370)); //12
      robotLocation.addPoint(30, -(335));    //13直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);        //14初期位置
      break;
    case RED_MIDDLE_FINAL_bathTowelLeft:
      whichMecha = left;
      robotLocation.addPoint(10, -(390));    //二本目のポール前
      robotLocation.addPoint((123), -(395)); //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((180), -(380)); //竿目印のところまで移動
      robotLocation.addPoint((180), -(390)); //ハサミつく位置まで前に
      robotLocation.addPoint((180), -(390)); //最初に直進
      robotLocation.addPoint((275), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint((275), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((275), -(370));
      robotLocation.addPoint(30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);     //初期位置
      break;
    case RED_MIDDLE_FINAL_bathTowelRight:
      whichMecha = right;
      robotLocation.addPoint(10, -(390));    //二本目のポール前
      robotLocation.addPoint((123), -(395)); //リミット接触前ちょこちょこ進む
      robotLocation.addPoint((275), -(380)); //竿目印のところまで移動
      robotLocation.addPoint((275), -(390)); //ハサミつく位置まで前に
      robotLocation.addPoint((275), -(390)); //最初に直進
      robotLocation.addPoint((375), -(400)); //洗濯物横に引っ張る
      robotLocation.addPoint((375), -(390)); //ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((375), -(370));
      robotLocation.addPoint(30, -(335)); //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);     //初期位置
      break;
    case RED_MIDDLE_FINAL_bathTowelboth:     //決勝(左ロジャー横掛け),(右ロジャー横掛け)
      whichMecha = left;                     //途中で変わる!!
      robotLocation.addPoint(10, -(390));    //1二本目のポール前
      robotLocation.addPoint((123), -(395)); //2リミット接触前ちょこちょこ進む
      robotLocation.addPoint((180), -(390)); //3竿目印のところまで移動
      robotLocation.addPoint((180), -(390)); //4ハサミつく位置まで前に
      robotLocation.addPoint((180), -(390)); //5最初に直進
      robotLocation.addPoint((275), -(400)); //6洗濯物横に引っ張る
      robotLocation.addPoint((275), -(390)); //7ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((275), -(390)); //8竿目印のところまで移動     2つ目！
      robotLocation.addPoint((275), -(390)); //9ハサミつく位置まで前に
      robotLocation.addPoint((375), -(400)); //10洗濯物横に引っ張る
      robotLocation.addPoint((375), -(385)); //11ロジャー降ろしながら後ろに引く
      robotLocation.addPoint((375), -(370)); //12
      robotLocation.addPoint(30, -(335));    //13直線移動できる位置まで戻ってくる
      robotLocation.addPoint(30, 30);        //14初期位置
      break;
    case RED_BACK_Sheets: //右ロジャー
      whichMecha = right;
      robotLocation.addPoint(10, -(500));
      robotLocation.addPoint((140), -(570));
      robotLocation.addPoint((170), -(590));
      robotLocation.addPoint((340), -(600)); //右リミットスイッチが接触しないから596に変更
      robotLocation.addPoint((395), -(575));
      robotLocation.addPoint(40, -(520));
      robotLocation.addPoint(30, 20);
      break;
  }
  for (int i = 0; i < 2; i++)
  {
    holder[i].grasp('r');
    holder[i].grasp('l');
  }
  robotLocation.sendNext();
  accelAlgorithm.setPositionChangedFlag();
  accelAlgorithm.setAllocateErrorCircleRadius(50);
  while (1)
  {
    //STLinkTerminal.printf("%.1lf %.1lf %.1lf   ", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
    //STLinkTerminal.printf("%.3lf\t%.3lf\t%.3lf\r\n", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
    //STLinkTerminal.printf("%d\t%d\t%d\t%d\t%d\t%d\t\r\n", (int)(pegAttacherPWM[0][0] * 100), (int)(pegAttacherPWM[0][1] * 100), (int)(hangerPWM[0][0] * 100), (int)(hangerPWM[0][1] * 100), (int)(rojarArmPWM[0][0] * 100), (int)(rojarArmPWM[0][1] * 100));
    /*static unsigned long int prevDisplayed = 0;
    if (((TimerForLCD.read_ms() - prevDisplayed) > 100)) //about 10Hz flash rate
    {
      LCDDriver.clear();
      LCDDriver.home();
      serialLCD.printf("posT:%d %d %d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
      LCDDriver.setCursor(2, 0);
      serialLCD.printf("posC:%.1lf %.1lf %.1lf   ", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
      LCDDriver.setCursor(3, 0);
      serialLCD.printf("pwm:%.2lf %.2lf %.2lf   ", driverPWMOutput[0], driverPWMOutput[1], driverPWMOutput[2]);
      LCDDriver.setCursor(4, 0);
      serialLCD.printf("vec:%.2lf %.2lf %.2lf   ", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
      prevDisplayed = TimerForLCD.read_ms();
    }*/
    static unsigned int wayPointSignature = 1;
    allUpdate();
    switch (currentRunningMode)
    {
      case RED_FRONT_PRE_bathTowelLeft:
        accelAlgorithm.setAllocateErrorCircleRadius(Robot.permitErrorCircleRadius);
        limitSwitchBar[left].update();
        if (limitSwitchBar[left].stats() && wayPointSignature == 1)
        {
          accelAlgorithm.setCurrentYPosition(-(190));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(190), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
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
              rojarArm[whichMecha].setHeight(455);
              limitSwitchBar[left].update();
              if (!limitSwitchBar[left].stats())
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
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
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
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp('r');
                  seq = 3;
                }
                if (2200 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  rojarArm[whichMecha].setHeight(0);
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
                    rojarArm[whichMecha].setHeight(900); //バスタオル後ろ巻いてしまうからめっちゃ高めに上げる　前：600
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
                rojarArm[whichMecha].setHeight(500);
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlag = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlag)
              {
                holder[whichMecha].half('l');
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
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              pegAttacher[whichMecha].reload();
              break;
            case 9:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holder[whichMecha].free('r');
              holder[whichMecha].free('l');
              while (1)
              {
                allUpdate();
                LCDDriver.setCursor(2, 0);
                serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
              }
              break;
            default:
              LCDDriver.clear();
              serialLCD.printf("WayPoint ERROR");
              while (1)
                ;
          }
        }
        break;
      case RED_MIDDLE_PRE_bathTowelRight:
      case RED_MIDDLE_FINAL_bathTowelRight:
        limitSwitchBar[left].update();
        if (limitSwitchBar[left].stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
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
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(1600); //押し出す前に大幅に上げる
              wayPointSignature++;
              break;
            case 2:
              limitSwitchBar[left].update();
              if (!limitSwitchBar[left].stats())
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
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
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
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp('r');
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
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlag = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlag)
              {
                holder[whichMecha].half('l');
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
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 10:
              holder[whichMecha].grasp('r');
              holder[whichMecha].grasp('l');
              pegAttacher[whichMecha].reload();
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 11:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holder[whichMecha].free('r');
              holder[whichMecha].free('l');
              while (1)
              {
                allUpdate();
                LCDDriver.setCursor(2, 0);
                serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
              }
              break;
            default:
              LCDDriver.clear();
              serialLCD.printf("WayPoint ERROR");
              while (1)
                ;
          }
        }
        break;
      case RED_MIDDLE_PRE_bathTowelLeft:
      case RED_MIDDLE_FINAL_bathTowelLeft:
        limitSwitchBar[left].update();
        if (limitSwitchBar[left].stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
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
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(2000); //押し出す前に大幅に上げる
              wayPointSignature++;
              break;
            case 2:
              limitSwitchBar[right].update();
              if (!limitSwitchBar[right].stats())
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
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                  rojarArm[whichMecha].setHeight(1570); //洗濯物離すために大幅に下げる
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
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp('r');
                  seq = 3;
                }
                if (3000 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  rojarArm[whichMecha].setHeight(1850);
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
                    rojarArm[whichMecha].setHeight(2400); //2100
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
                rojarArm[whichMecha].setHeight(1900);
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlag = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlag)
              {
                holder[whichMecha].half('l');
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
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 10:
              holder[whichMecha].grasp('r');
              holder[whichMecha].grasp('l');
              pegAttacher[whichMecha].reload();
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 11:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holder[whichMecha].free('r');
              holder[whichMecha].free('l');
              while (1)
              {
                allUpdate();
                LCDDriver.setCursor(2, 0);
                serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
              }
              break;
            default:
              LCDDriver.clear();
              serialLCD.printf("WayPoint ERROR");
              while (1)
                ;
          }
        }
        break;
      case RED_MIDDLE_PRE_bathTowelboth:
      case RED_MIDDLE_FINAL_bathTowelboth:
        limitSwitchBar[left].update();
        if (limitSwitchBar[left].stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(390));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(390), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
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
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              rojarArm[whichMecha].setHeight(2000);
              wayPointSignature++;
              break;
            case 2:
              limitSwitchBar[left].update();
              if (!limitSwitchBar[left].stats())
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
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hanger[whichMecha].update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hanger[whichMecha].stats() && !initialHangerFlag && !hangerHasDoneFlagLeft)
                {
                  hangerHasDoneFlagLeft = 1;
                  hanger[whichMecha].setLength(0);
                  hanger[whichMecha].update();
                  rojarArm[whichMecha].setHeight(1570); //洗濯物離すために大幅に下げる
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
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp('r');
                  seq = 3;
                }
                if (3000 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhaseLeft = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                  rojarArm[whichMecha].setHeight(1850);
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
                    rojarArm[whichMecha].setHeight(2400);
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
              OmniKinematics.setMaxPWM(0.17);
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
                rojarArm[whichMecha].setHeight(1900);
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlag = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlag)
              {
                holder[whichMecha].half('l');
                wayPointSignature++;
                clothHangerTimer.start();
              }
              break;
            case 8:
              rojarArm[whichMecha].setHeight(0);

              if (650 < clothHangerTimer.read_ms())
              {
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                holder[whichMecha].grasp('l');
                whichMecha = !whichMecha; //途中で機構入れ替え！！
                rojarArm[whichMecha].setHeight(1600);
                robotLocation.sendNext(); //8
                accelAlgorithm.setPositionChangedFlag();
                wayPointSignature++;
              }
              break;
            case 9:
              static int armPhaseRight = 1, hangerHasDoneFlagRight = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArm[whichMecha].stats() && armPhaseRight == 1)   //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
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
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holder[whichMecha].grasp('r');
                  seq = 3;
                }
                if (2200 < clothHangerTimer.read_ms() && seq == 3)
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
            case 10:
              if (rojarArm[whichMecha].stats())
              {
                wayPointSignature++;
              }
              break;
            case 11:
              OmniKinematics.setMaxPWM(0.17);
              robotLocation.sendNext(); //横移動 10
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 12:
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
                rojarArm[whichMecha].setHeight(1530);
                rojarArm[whichMecha].update();
                clothHangerTimer.stop();
                clothHangerTimer.reset();
                releasedFlagSec = 1;
              }
              if (rojarArm[whichMecha].stats() && releasedFlagSec)
              {
                holder[whichMecha].half('l');
                wayPointSignature++;
                clothHangerTimer.start();
              }
              break;
            case 13:
              if (650 < clothHangerTimer.read_ms())
              {
                robotLocation.sendNext(); //12
                accelAlgorithm.setPositionChangedFlag();
                wayPointSignature++;
                clothHangerTimer.stop();
                clothHangerTimer.reset();
              }
              break;
            case 14:
              rojarArm[whichMecha].setHeight(0);
              robotLocation.sendNext(); //13
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 15:
              holder[right].grasp('r');
              holder[right].grasp('l');
              holder[left].grasp('r');
              holder[left].grasp('l');
              pegAttacher[right].reload();
              pegAttacher[left].reload();
              robotLocation.sendNext(); //14
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;
            case 16:
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              holder[right].free('r');
              holder[right].free('l');
              holder[left].free('r');
              holder[left].free('l');
              while (1)
              {
                allUpdate();
                LCDDriver.setCursor(2, 0);
                serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
              }
              break;
            default:
              LCDDriver.clear();
              serialLCD.printf("WayPoint ERROR");
              while (1)
                ;
          }
        }
        break;
      case RED_BACK_Sheets:
        limitSwitchBar[left].update();
        if (limitSwitchBar[left].stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(590));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(590), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 200)
          {
            IMU.setYaw(0);
            wayPointSignature++;
          }
          flag++;
        }

        limitSwitchSide[whichMecha].update();
        if (limitSwitchSide[whichMecha].stats() && wayPointSignature == 5)
        {
          accelAlgorithm.setCurrentXPosition(395);
          robotLocation.setCurrentPoint(395, robotLocation.getYLocationData(), robotLocation.getYawStatsData());
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
              rojarArm[whichMecha].setHeight(2060); //2100-洗濯物干し最適高さ , 2850-洗濯バサミ最適高さ
              wayPointSignature++;
              break;

            case 2:
              limitSwitchBar[left].update();
              if (!limitSwitchBar[left].stats())
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
                  hanger[whichMecha].setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
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
                  holder[whichMecha].release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1900 && seq == 2)
                {
                  holder[whichMecha].grasp('r');
                  rojarArm[whichMecha].setHeight(2500);
                  rojarArm[whichMecha].update();
                  seq = 3;
                }
                if (seq == 3)
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
                //accelAlgorithm.setMaxOutput(Robot.estimateDriveMaxPWM);
                //accelAlgorithm.setMinOutput(Robot.estimateDriveMinPWM);
                robotLocation.setCurrentPoint(robotLocation.getXLocationData() + 10, robotLocation.getYLocationData(), robotLocation.getYawStatsData());
                break;
              }
              break;

            case 6:
              static bool initialFlag = 1;
              if (initialFlag)
              {
                robotLocation.sendNext();
                accelAlgorithm.setPositionChangedFlag();
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
              holder[whichMecha].release('l');
              rojarArm[whichMecha].setMaxPWM(Robot.estimateRojarArmMaxPWM);
              rojarArm[whichMecha].setHeight(0);
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;

            case 8:
              holder[whichMecha].grasp('r');
              holder[whichMecha].grasp('l');
              pegAttacher[whichMecha].reload();
              robotLocation.sendNext();
              accelAlgorithm.setPositionChangedFlag();
              wayPointSignature++;
              break;

            case 9:
              holder[whichMecha].free('r');
              holder[whichMecha].free('l');
              holder[!whichMecha].free('r');
              holder[!whichMecha].free('l');
              LCDDriver.clear();
              serialLCD.printf("TASKS CLOSED");
              while (1)
              {
                allUpdate();
                LCDDriver.setCursor(2, 0);
                serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
              }
              break;

            default:
              LCDDriver.clear();
              serialLCD.printf("WayPoint ERROR");
              while (1)
              {
              }
          }
        }
    }
  }

#endif //GAME_MODECHANGE_ONBOARDSWITCH

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
  while (1)
  {
    startButton.update();
    if (startButton.stats() == 1)
      break;
  }
  while (1)
  {
    holder[0].grasp('r');
    holder[0].grasp('l');
    holder[1].grasp('r');
    holder[1].grasp('l');
    wait(2.5);
    holder[0].release('r');
    holder[0].release('l');
    holder[1].release('r');
    holder[1].release('l');
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

/*int getRunningMode()
{
  int UIFData[1]; //sequence number
  if (!UIF.receive(&UIFData[0]))
  {
    return 0;
  }
  int ACK = 0x06;
  UIF.transmit(1, &ACK); //モード受けとり確認用シグナル
  return UIFData[0];
}*/
