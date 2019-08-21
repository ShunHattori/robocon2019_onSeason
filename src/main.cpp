#include "DriveSource\DriveTrain.h"
#include "DriveSource\LocationManager.h"
#include "DriveSource\MWodometry.h"
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

/*
    新型足回り制御テスト用コード
 */
//#define TEST_DRIVE_NEWTYPE

/*
    ゲーム用新型足回り制御
 */
//#define GAME_DRIVE_NEWTYPE            //ロボットの動作は2mシーツ用
#define GAME_MODECHANGE_ONBOARDSWITCH //基板上の青スイッチで動作切り替え（シーツ・バスタオル）
//#define TEST_DRIVE

//#define MECA_CLASS_DEBUG
//#define TEST_RojarArm_UpDownLoop
//#define TEST_PEGLaunch      //tested on 6/20(Thur)
//#define TEST_HangerMovement //tested on 6/20(Thur)
//#define TEST_HoldServo
//#define TEST_RojarArmUpDownOnce
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
  const double permitErrorCircleRadius = 2.0;
  const int decreaseSpeedCircleRadius = 70;
  const double estimateDriveMaxPWM = 0.4; // max:0.7, recommend:0.64 //DEFAULT 0.5
  const double estimateDriveMinPWM = 0.2;
  const double estimatePegMaxPWM = 0.6;
  const double estimateHangerMaxPWM = 0.6;
  const double estimateRojarArmMaxPWM = 0.96;
  const double PegVoltageImpressionTime = 0.2;

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

#include "DriveSource\OmniKinematics3WD.h"
OmniKinematics3WD OmniKinematics;
double driverPWMOutput[3] = {};

Timer TimerForQEI;
MPU9250 IMU;
QEI encoderXAxis(OdometryPin.XAxisAPulse, OdometryPin.XAxisBPulse, OdometryPin.XAxisIndexPulse, Robot.encoderPPRHigh, &TimerForQEI, QEI::X4_ENCODING);
QEI encoderYAxis(OdometryPin.YAxisAPulse, OdometryPin.YAxisBPulse, OdometryPin.YAxisIndexPulse, Robot.encoderPPRHigh, &TimerForQEI, QEI::X4_ENCODING);
MWodometry odometryXAxis(encoderXAxis, Robot.encoderPPRHigh, Robot.encoderAttachedWheelRadius);
MWodometry odometryYAxis(encoderYAxis, Robot.encoderPPRHigh, Robot.encoderAttachedWheelRadius);
LocationManager<double> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometryXAxis, odometryYAxis, IMU, Robot.permitErrorCircleRadius, Robot.decreaseSpeedCircleRadius);

Serial STLinkTerminal(USBTX, USBRX, SerialBaud.HardwareSerial); //Surfaceのターミナルとの通信用ポート
DigitalOut IMUisReadyLED(LED1);                                 //IMUセンサキャリブレーション完了表示用LED用デジタル出力

int main(void)
{

#ifdef GAME_DRIVE_NEWTYPE
  Serial serialLCD(PC_6, NC, SerialBaud.LCD);
  NewHavenDisplay LCDDriver(serialLCD);
  Timer TimerForLCD;
  Timer clothHangerTimer;
  TimerForLCD.start();
  serialLCD.printf("WAITING...");
  DebounceSwitch startButton(PG_2, 'U'); //create object using pin "PG_2" with PullUpped
  DebounceSwitch limitSwitchBarFront(PF_14, 'U');
  int initialButtonPressToken = 1, startButtonPressedFlag = 0;
  ClothHold holder(PE_5, PE_6); //right,leftServo //PE_5, PE_6
  holder.free('r');
  holder.free('l');
  Peg pegAttacher(PC_9, PC_8, 0.6, 0.2); //pin ,pin pwm, time
  ClothHang hanger(PF_8, PA_0);          //PF_8, PA_0
  hanger.setMaxPWM(0.6);                 //0.85
  QEI clothHangEncoder(PE_2, PD_11, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
  RojarArm rojarArmRight(PF_7, PF_9); //PF_7, PF_9->pe10
  rojarArmRight.setMaxPWM(0.96);      //0.92
  QEI rojarArmRightEncoder(PG_0, PD_1, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
  int wayPointSignature = 1;
  IMU.setup(PB_9, PB_8);
  IMUisReadyLED.write(1);
  accelAlgorithm.setMaxOutput(ESTIMATE_MAX_PWM);
  accelAlgorithm.setMinOutput(ESTIMATE_MIN_PWM);
  accelAlgorithm.setAllocateErrorCircleRadius(60);
  OmniKinematics.setMaxPWM(ESTIMATE_MAX_PWM);
  driveWheel.setMaxPWM(ESTIMATE_MAX_PWM);
  robotLocation.addPoint(0, -500, 0);
  robotLocation.addPoint(112, -565, 0);
  //robotLocation.addPoint(112, -583, 0); 少し進むところをリミットスイッチを利用した位置合わせに変更
  robotLocation.addPoint(340, -585, 0); //5cm手前
  robotLocation.addPoint(340, -575, 0);
  robotLocation.addPoint(0, -520, 0);
  robotLocation.addPoint(-10, 5, 0);
  while (1)
  {
    if (initialButtonPressToken)
    {
      startButton.update();
      if (startButton.stats() && initialButtonPressToken)
      {
        initialButtonPressToken = 0;
        robotLocation.sendNext(); //機構テスト時はコメントアウト
        holder.grasp('r');
        holder.grasp('l');
      }
    }
    rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
    rojarArmRight.update();
    pegAttacher.update();
    hanger.setEncoderPulse(clothHangEncoder.getPulses());
    hanger.update();
    accelAlgorithm.update();
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), output);
    //display current robot vectors (3-Axis) and calculated PWMs
    //STLinkTerminal.printf("CurrentVector:%.1lf %.1lf %.1lf  \t", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
    //STLinkTerminal.printf("CalculatedPWM:%.2f %.2f %.2f \r\n", output[0], output[1], output[2]);
    driveWheel.apply(output);
    static unsigned long int prevDisplayed = 0;
    if (((TimerForLCD.read_ms() - prevDisplayed) > 100) && !initialButtonPressToken) //about 24Hz flash rate
    {
      LCDDriver.clear();
      LCDDriver.home();
      serialLCD.printf("%d %d %d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
      LCDDriver.setCursor(2, 0);
      serialLCD.printf("%.1lf %.1lf %.1lf   ", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
      prevDisplayed = TimerForLCD.read_ms();
    }
    /*if (!initialButtonPressToken) //機構テスト
           {
            static int hangerHasDoneFlag = 0, initialHangerFlag = 1;
            if (initialHangerFlag) //ロジャー展開後初めての処理
            {
                hanger.setLength(1000); //洗濯物掛ける
                hanger.update();        //あとのstats判定のために一度状態を更新する
                initialHangerFlag = 0;
            }
            if (hanger.stats() && !initialHangerFlag && !hangerHasDoneFlag)
            {
                hangerHasDoneFlag = 1;
                hanger.setLength(0);
                hanger.update();
            }
            if (hanger.stats() && hangerHasDoneFlag) //洗濯物が竿にかかっているだけの状態
            {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1500 && seq == 1)
                {
                    holder.release('r');
                    seq = 2;
                }
                if (1500 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 3000 && seq == 2)
                {
                    holder.grasp('r');
                    seq = 3;
                }
                if (3000 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 4000 && seq == 3)
                {
                    pegAttacher.launch();
                    seq = 4;
                }
                if (4000 < clothHangerTimer.read_ms() && seq == 4)
                {
                    //robotLocation.sendNext(); //次の座標を送信
                    //wayPointSignature++;
                }
            }
           }*/
    limitSwitchBarFront.update();
    if (limitSwitchBarFront.stats() && wayPointSignature == 2)
    {
      accelAlgorithm.setCurrentYPosition(-580);
      robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -580, robotLocation.getYawStatsData());
      static int flag = 0;
      if (flag == 2000)
        wayPointSignature++;
      flag++;
    }

    if (robotLocation.checkMovingStats(accelAlgorithm.getStats()) && !initialButtonPressToken)
    {
      switch (wayPointSignature)
      {
        case 1:
          accelAlgorithm.setAllocateErrorCircleRadius(3.65);
          robotLocation.sendNext();      //三本目のポール少し手前の位置
          rojarArmRight.setHeight(2600); //2100-洗濯物干し最適高さ , 2850-洗濯バサミ最適高さ
          wayPointSignature++;
          break;

        case 2:
          //static int savedXLocation = accelAlgorithm.getCurrentXPosition();
          limitSwitchBarFront.update();
          if (!limitSwitchBarFront.stats())
          {
            robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
            break;
          }
          break;

        case 3:
          static int armPhase = 1, hangerHasDoneFlag = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
          if (rojarArmRight.stats() && armPhase == 1)     //ロジャーアーム展開完了
          {
            static int initialHangerFlag = 1;
            if (initialHangerFlag) //ロジャー展開後初めての処理
            {
              hanger.setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
              hanger.update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
              initialHangerFlag = 0;
            }
            if (hanger.stats() && !initialHangerFlag && !hangerHasDoneFlag)
            {
              hangerHasDoneFlag = 1;
              hanger.setLength(0);
              hanger.update();
            }
          }
          static int initialChangeHeightFlag = 1;
          if (hanger.stats() && hangerHasDoneFlag && initialChangeHeightFlag)
          {
            static unsigned int seq = 1, timerStartFlag = 1;
            if (timerStartFlag)
            {
              clothHangerTimer.start();
              timerStartFlag = 0;
            }
            if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1300 && seq == 1)
            {
              holder.release('r');
              seq = 2;
            }
            if (1300 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1900 && seq == 2)
            {
              holder.center('r');
              rojarArmRight.setHeight(3200);
              rojarArmRight.update();
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
          if (rojarArmRight.stats() && armPhase == 2)
          {
            if (hanger.stats() && hangerHasDoneFlag) //洗濯物が竿にかかっているだけの状態
            {
              static unsigned int seq = 1, timerStartFlag = 1;
              if (timerStartFlag)
              {
                clothHangerTimer.start();
                timerStartFlag = 0;
              }
              if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1) //`hutatume 1700ms
              {
                pegAttacher.launch();
                seq = 2;
              }
              if (100 < clothHangerTimer.read_ms() && seq == 2)
              {
                robotLocation.sendNext(); //次の座標を送信
                wayPointSignature++;
              }
            }
          }
          break;
        case 4:
          static bool initialFlag = 1;
          if (initialFlag)
          {
            robotLocation.sendNext();
            rojarArmRight.setMaxPWM(0.5);
            rojarArmRight.setHeight(2420);
            rojarArmRight.update();
            initialFlag = 0;
          }
          if (rojarArmRight.stats())
          {
            wayPointSignature++;
          }
          break;
        case 5:
          holder.release('l');
          rojarArmRight.setHeight(0); //ロジャーアーム縮小
          //pegAttacher.reload();
          accelAlgorithm.setAllocateErrorCircleRadius(30);
          robotLocation.sendNext();
          wayPointSignature++;
          break;

        case 6:
          accelAlgorithm.setAllocateErrorCircleRadius(3.7);
          holder.grasp('r');
          holder.grasp('l');
          robotLocation.sendNext();
          wayPointSignature++;
          break;

        case 7:
          if (rojarArmRight.stats())
          {
            wayPointSignature++;
          }
          break;
        case 8:
          LCDDriver.clear();
          serialLCD.printf("TASKS CLOSED");
          holder.free('r');
          holder.free('l');
          while (1)
          {
            LCDDriver.setCursor(2, 0);
            accelAlgorithm.update();
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
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
  while (1)
  {
    /* code */
  }

#endif //GAME_DRIVE_NEWTYPE

#ifdef GAME_MODECHANGE_ONBOARDSWITCH
  enum
  {
    bathTowel,
    Sheets,
  };

  struct
  {
    PinName LCD1TX = NC;
    PinName LCD1RX = NC;
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
    PinName frontR = PE_10;
    PinName frontL = PE_12;
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
    PinName holderRightServoR = PE_5;
    PinName holderRightServoL = PE_6;
    PinName holderLeftServoR = PF_8;
    PinName holderLeftServoL = PF_7;
  } MecaPin;

  UnitProtocol UIF(serialDevice.UIFTX, serialDevice.UIFRX, SerialBaud.HardwareSerial);
  UnitProtocol MDD1(serialDevice.MDD1TX, serialDevice.MDD1RX, SerialBaud.HardwareSerial);
  UnitProtocol MDD2(serialDevice.MDD2TX, serialDevice.MDD2RX, SerialBaud.HardwareSerial);
  UnitProtocol MDD3(serialDevice.MDD3TX, serialDevice.MDD3RX, SerialBaud.HardwareSerial);

  //Serial serialLCD(serialDevice.LCD1TX, serialDevice.LCD1RX, SerialBaud.LCD);
  //NewHavenDisplay LCDDriver(serialLCD);

  DebounceSwitch onBoardSwitch(USER_BUTTON, 'd');
  DebounceSwitch startButton(Switch.toBegin, 'U'); //create object using pin "PG_2" with PullUpped
  DebounceSwitch limitSwitchBarFrontRight(Switch.frontR, 'U');
  DebounceSwitch limitSwitchBarFrontLeft(Switch.frontL, 'U');

  DigitalOut modeIndicatorLED1(LED2);
  DigitalOut modeIndicatorLED2(LED3);

  Timer TimerForLCD;
  Timer clothHangerTimer;
  QEI clothHangRightEncoder(MecaPin.clothHangRightEncoderAPulse, MecaPin.clothHangRightEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING);
  QEI clothHangLeftEncoder(MecaPin.clothHangLeftEncoderAPulse, MecaPin.clothHangLeftEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING);

  QEI rojarArmRightEncoder(MecaPin.rojarArmRightEncoderAPulse, MecaPin.rojarArmRightEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING);
  QEI RojarArmLeftEncoder(MecaPin.rojarArmLeftEncoderAPulse, MecaPin.rojarArmLeftEncoderBPulse, NC, Robot.encoderPPRLow, &TimerForQEI, QEI::X4_ENCODING);

  ClothHold holderRight(MecaPin.holderRightServoR, MecaPin.holderRightServoL); //right,leftServo
  ClothHold holderLeft(MecaPin.holderLeftServoR, MecaPin.holderLeftServoL);    //right,leftServo

  static double pegAttacherPWM[2][2];                                                               //right(CW,CCW),left(CW,CCW)
  Peg pegAttacherRight(Robot.estimatePegMaxPWM, Robot.PegVoltageImpressionTime, pegAttacherPWM[0]); //pwm, time
  Peg pegAttacherLeft(Robot.estimatePegMaxPWM, Robot.PegVoltageImpressionTime, pegAttacherPWM[1]);  //pwm, time

  static double hangerPWM[2][2]; //right(CW,CCW),left(CW,CCW)
  ClothHang hangerRight(hangerPWM[0]);
  ClothHang hangerLeft(hangerPWM[1]);

  static double rojarArmPWM[2][2]; //right(CW,CCW),left(CW,CCW)
  RojarArm rojarArmRight(rojarArmPWM[0]);
  RojarArm rojarArmLeft(rojarArmPWM[1]);

  hangerRight.setMaxPWM(Robot.estimateHangerMaxPWM);
  rojarArmRight.setMaxPWM(Robot.estimateRojarArmMaxPWM);
  accelAlgorithm.setMaxOutput(Robot.estimateDriveMaxPWM);
  accelAlgorithm.setMinOutput(Robot.estimateDriveMinPWM);
  OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
  holderRight.free('r');
  holderRight.free('l');
  IMU.setup(I2CPin.IMUSDA, I2CPin.IMUSCL); //sda scl
  IMUisReadyLED.write(1);
  //TimerForLCD.start();
  //serialLCD.printf("WAITING...");

  static int currentRunningMode;
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
    if (onBoardSwitch.stats() && stateHasChanged)
    {
      pushedCounter++;
      currentRunningMode = (pushedCounter % 2) == 0 ? bathTowel : Sheets;
    }
    previousState = onBoardSwitch.stats();
    if (currentRunningMode)
    {
      modeIndicatorLED1.write(1);
      modeIndicatorLED2.write(0);
    }
    else
    {
      modeIndicatorLED1.write(0);
      modeIndicatorLED2.write(1);
    }
    if (startButton.stats())
      break;
  }

  switch (currentRunningMode)
  {
    case bathTowel:                                    //縦:112-170,横112-205
      robotLocation.addPoint(0, -(340 + 47));          //二本目のポール前
      robotLocation.addPoint((112 + 35), -(365 + 47)); //さらに近づいてリミット監視開始
      robotLocation.addPoint((135 + 35), -(380 + 47));
      robotLocation.addPoint((230 + 35), -(385 + 47)); //横移動
      robotLocation.addPoint(10, -(335 + 47));         //直線移動できる位置まで戻ってくる
      robotLocation.addPoint(0, 0);                    //初期位置
      break;
    case Sheets:
      robotLocation.addPoint(0, -(500 + 47), 0);
      robotLocation.addPoint((112 + 35), -(565 + 47), 0);
      robotLocation.addPoint((340 + 35), -(585 + 47), 0);
      robotLocation.addPoint((340 + 35), -(575 + 47), 0);
      robotLocation.addPoint(0, -(520 + 47), 0);
      robotLocation.addPoint(0, 0, 0);
      break;
  }
  holderRight.grasp('r');
  holderRight.grasp('l');
  robotLocation.sendNext();
  accelAlgorithm.setAllocateErrorCircleRadius(20);
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
      serialLCD.printf("%d %d %d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
      LCDDriver.setCursor(2, 0);
      serialLCD.printf("%.1lf %.1lf %.1lf   ", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
      prevDisplayed = TimerForLCD.read_ms();
    }*/
    static unsigned int wayPointSignature = 1, initialButtonPressToken = 1;
    rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
    hangerRight.setEncoderPulse(clothHangRightEncoder.getPulses());
    rojarArmRight.update();
    pegAttacherRight.update();
    hangerRight.update();
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
    int MDD2Packet[6] = {
        (int)(rojarArmPWM[0][1] * 100),
        (int)(rojarArmPWM[0][0] * 100),
        (int)(hangerPWM[0][0] * 100),
        (int)(hangerPWM[0][1] * 100),
        (int)(pegAttacherPWM[0][1] * 100),
        (int)(pegAttacherPWM[0][0] * 100),
    };
    MDD1.transmit(6, MDD1Packet);
    MDD2.transmit(6, MDD2Packet);
    switch (currentRunningMode)
    {
      case bathTowel:
        limitSwitchBarFrontRight.update();
        if (limitSwitchBarFrontRight.stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(380 + 47));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(380 + 47), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
          {
            robotLocation.sendNext();
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
              accelAlgorithm.setAllocateErrorCircleRadius(3.5);
              robotLocation.sendNext();
              rojarArmRight.setHeight(1600);
              wayPointSignature++;
              break;
            case 2:
              limitSwitchBarFrontRight.update();
              if (!limitSwitchBarFrontRight.stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
                break;
              }
              break;
            case 3:
              static int armPhase = 1, hangerHasDoneFlag = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArmRight.stats() && armPhase == 1)     //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hangerRight.setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hangerRight.update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hangerRight.stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hangerRight.setLength(0);
                  hangerRight.update();
                }
              }
              if (hangerRight.stats() && hangerHasDoneFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && seq == 1)
                {
                  holderRight.release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && seq == 2)
                {
                  holderRight.center('r');
                  seq = 3;
                }
                if (1900 < clothHangerTimer.read_ms() && seq == 3)
                {
                  armPhase = 2;
                  clothHangerTimer.stop();
                  clothHangerTimer.reset();
                }
              }
              if (armPhase == 2)
              {
                if (hangerRight.stats()) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1)
                  {
                    pegAttacherRight.launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    robotLocation.sendNext();
                    accelAlgorithm.setMaxOutput(0.25); //引っ張るときはゆっくり
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 4:
              accelAlgorithm.setMaxOutput(Robot.estimateDriveMaxPWM);
              holderRight.release('l');
              rojarArmRight.setHeight(0);
              robotLocation.sendNext();
              wayPointSignature++;
              break;
            case 5:
              holderRight.grasp('r');
              holderRight.grasp('l');
              robotLocation.sendNext();
              wayPointSignature++;
              break;
            case 6:
              //LCDDriver.clear();
              //serialLCD.printf("TASKS CLOSED");
              holderRight.free('r');
              holderRight.free('l');
              while (1)
              {
                //LCDDriver.setCursor(2, 0);
                accelAlgorithm.update();
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
                //ここにMMD送信(drive)
                int MDD1Packet[6] = {
                    driverPWMOutput[0] < 0 ? 0 : (int)(driverPWMOutput[0] * 100),
                    driverPWMOutput[0] > 0 ? 0 : -(int)(driverPWMOutput[0] * 100),
                    driverPWMOutput[1] < 0 ? 0 : (int)(driverPWMOutput[1] * 100),
                    driverPWMOutput[1] > 0 ? 0 : -(int)(driverPWMOutput[1] * 100),
                    driverPWMOutput[2] < 0 ? 0 : (int)(driverPWMOutput[2] * 100),
                    driverPWMOutput[2] > 0 ? 0 : -(int)(driverPWMOutput[2] * 100),
                };
                MDD1.transmit(6, MDD1Packet);
                //serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
              }
              break;
            default:
              //LCDDriver.clear();
              //serialLCD.printf("WayPoint ERROR");
              while (1)
                ;
          }
        }
        break;

      case Sheets:
        //display current robot vectors (3-Axis) and calculated PWMs
        //STLinkTerminal.printf("CurrentVector:%.1lf %.1lf %.1lf  \t", accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector());
        //STLinkTerminal.printf("CalculatedPWM:%.2f %.2f %.2f \r\n", output[0], output[1], output[2]);
        if (initialButtonPressToken)
        {
          startButton.update();
          if (startButton.stats() && initialButtonPressToken)
          {
            initialButtonPressToken = 0;
            robotLocation.sendNext(); //機構テスト時はコメントアウト
            holderRight.grasp('r');
            holderRight.grasp('l');
          }
        }
        rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
        rojarArmRight.update();
        pegAttacherRight.update();
        hangerRight.setEncoderPulse(clothHangRightEncoder.getPulses());
        hangerRight.update();
        accelAlgorithm.update();
        accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
        OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), driverPWMOutput);
        /*static unsigned long int prevDisplayed = 0;
        if (((TimerForLCD.read_ms() - prevDisplayed) > 100) && !initialButtonPressToken) //about 24Hz flash rate
        {
          LCDDriver.clear();
          LCDDriver.home();
          serialLCD.printf("%d %d %d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
          LCDDriver.setCursor(2, 0);
          serialLCD.printf("%.1lf %.1lf %.1lf   ", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
          prevDisplayed = TimerForLCD.read_ms();
        }*/
        limitSwitchBarFrontLeft.update();
        if (limitSwitchBarFrontLeft.stats() && wayPointSignature == 2)
        {
          accelAlgorithm.setCurrentYPosition(-(580 + 47));
          robotLocation.setCurrentPoint(robotLocation.getXLocationData(), -(580 + 47), robotLocation.getYawStatsData());
          static int flag = 0;
          if (flag == 100)
            wayPointSignature++;
          flag++;
        }

        if (robotLocation.checkMovingStats(accelAlgorithm.getStats()) && !initialButtonPressToken)
        {
          switch (wayPointSignature)
          {
            case 1:
              accelAlgorithm.setAllocateErrorCircleRadius(3.65);
              robotLocation.sendNext();      //三本目のポール少し手前の位置
              rojarArmRight.setHeight(2600); //2100-洗濯物干し最適高さ , 2850-洗濯バサミ最適高さ
              wayPointSignature++;
              break;

            case 2:
              //static int savedXLocation = accelAlgorithm.getCurrentXPosition();
              limitSwitchBarFrontLeft.update();
              if (!limitSwitchBarFrontLeft.stats())
              {
                robotLocation.setCurrentPoint(robotLocation.getXLocationData(), robotLocation.getYLocationData() - 5, robotLocation.getYawStatsData());
                break;
              }
              break;

            case 3:
              static int armPhase = 1, hangerHasDoneFlag = 0; //phase1=洗濯物掛ける, phase2=洗濯ばさみつける
              if (rojarArmRight.stats() && armPhase == 1)     //ロジャーアーム展開完了
              {
                static int initialHangerFlag = 1;
                if (initialHangerFlag) //ロジャー展開後初めての処理
                {
                  hangerRight.setLength(1200); //洗濯物掛ける //1000 for test , 1600 for max lenght, recommend:1530
                  hangerRight.update();        //すぐ下のstats判定のために一度状態を更新し判定フラグを未完了に設定する
                  initialHangerFlag = 0;
                }
                if (hangerRight.stats() && !initialHangerFlag && !hangerHasDoneFlag)
                {
                  hangerHasDoneFlag = 1;
                  hangerRight.setLength(0);
                  hangerRight.update();
                }
              }
              static int initialChangeHeightFlag = 1;
              if (hangerRight.stats() && hangerHasDoneFlag && initialChangeHeightFlag)
              {
                static unsigned int seq = 1, timerStartFlag = 1;
                if (timerStartFlag)
                {
                  clothHangerTimer.start();
                  timerStartFlag = 0;
                }
                if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1300 && seq == 1)
                {
                  holderRight.release('r');
                  seq = 2;
                }
                if (1300 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 1900 && seq == 2)
                {
                  holderRight.center('r');
                  rojarArmRight.setHeight(3200);
                  rojarArmRight.update();
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
              if (rojarArmRight.stats() && armPhase == 2)
              {
                if (hangerRight.stats() && hangerHasDoneFlag) //洗濯物が竿にかかっているだけの状態
                {
                  static unsigned int seq = 1, timerStartFlag = 1;
                  if (timerStartFlag)
                  {
                    clothHangerTimer.start();
                    timerStartFlag = 0;
                  }
                  if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 100 && seq == 1) //`hutatume 1700ms
                  {
                    pegAttacherRight.launch();
                    seq = 2;
                  }
                  if (100 < clothHangerTimer.read_ms() && seq == 2)
                  {
                    robotLocation.sendNext(); //次の座標を送信
                    wayPointSignature++;
                  }
                }
              }
              break;
            case 4:
              static bool initialFlag = 1;
              if (initialFlag)
              {
                robotLocation.sendNext();
                rojarArmRight.setMaxPWM(0.5);
                rojarArmRight.setHeight(2420);
                rojarArmRight.update();
                initialFlag = 0;
              }
              if (rojarArmRight.stats())
              {
                wayPointSignature++;
              }
              break;
            case 5:
              holderRight.release('l');
              rojarArmRight.setHeight(0); //ロジャーアーム縮小
              //pegAttacher.reload();
              accelAlgorithm.setAllocateErrorCircleRadius(30);
              robotLocation.sendNext();
              wayPointSignature++;
              break;

            case 6:
              accelAlgorithm.setAllocateErrorCircleRadius(3.7);
              holderRight.grasp('r');
              holderRight.grasp('l');
              robotLocation.sendNext();
              wayPointSignature++;
              break;

            case 7:
              if (rojarArmRight.stats())
              {
                wayPointSignature++;
              }
              break;
            case 8:
              holderRight.free('r');
              holderRight.free('l');
              //LCDDriver.clear();
              //serialLCD.printf("TASKS CLOSED");
              while (1)
              {
                accelAlgorithm.update();
                accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
                //LCDDriver.setCursor(2, 0);
                //serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
              }
              break;
            default:
              //LCDDriver.clear();
              //serialLCD.printf("WayPoint ERROR");
              while (1)
              {
              }
          }
        }
        break;
    }
  }

#endif //GAME_MODECHANGE_ONBOARDSWITCH

#ifdef TEST_DRIVE

  struct
  {
    PinName LCD1TX = PC_6;
    PinName LCD1RX = NC;
  } serialDevice;

  struct
  {
    PinName toBegin = PG_2;
  } Switch;

  DebounceSwitch startButton(Switch.toBegin, 'U'); //create object using pin "PG_2" with PullUpped
  accelAlgorithm.setMaxOutput(Robot.estimateDriveMaxPWM);
  accelAlgorithm.setMinOutput(Robot.estimateDriveMinPWM);
  OmniKinematics.setMaxPWM(Robot.estimateDriveMaxPWM);
  driveWheel.setMaxPWM(Robot.estimateDriveMaxPWM);
  robotLocation.addPoint(0, -520, 0);
  robotLocation.addPoint(300, -520, 0);
  robotLocation.addPoint(250, -300, 0);
  robotLocation.addPoint(0, 0, 0);
  IMU.setup(PB_9, PB_8);
  IMUisReadyLED.write(1);
  while (1) //モードセレクト処理
  {
    static bool stateHasChanged, previousState;
    static unsigned int pushedCounter;
    startButton.update();
    if (startButton.stats())
      break;
  }
  while (1)
  {
    static unsigned int wayPointSignature = 0;
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    accelAlgorithm.update();
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), output);
    driveWheel.apply(output);
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
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), output);
    driveWheel.apply(output);
    STLinkTerminal.printf("%4.4lf,%4.4lf,%4.4lf\r\n", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), accelAlgorithm.getCurrentYawPosition());
  }
#endif

#ifdef TEST_DRIVE_NEWTYPE
  Serial serialLCD(PC_6, NC, 9600);
  NewHavenDisplay LCDDriver(serialLCD);
  Timer LCDtimer;
  LCDtimer.start();
  serialLCD.printf("WAITING...");
  DigitalIn startButton(PG_2);
  startButton.mode(PullUp);
  int initialButtonPressToken = 1, startButtonPressedFlag = 0;
  ClothHold holder(PE_5, PE_6); //right,leftServo
  holder.release('r');
  holder.release('l');
  int wayPointSignature = 0;
  STLinkTerminal.baud(9600);
  IMU.setup(PB_9, PB_8);
  IMUisReadyLED.write(1);
  accelAlgorithm.setMaxOutput(ESTIMATE_MAX_PWM);
  accelAlgorithm.setMinOutput(ESTIMATE_MIN_PWM);
  OmniKinematics.setMaxPWM(ESTIMATE_MAX_PWM);
  robotLocation.addPoint(0, -100, 0);
  robotLocation.addPoint(100, -100, 0);
  robotLocation.addPoint(0, 0, 0);
  while (1)
  {
    if (initialButtonPressToken)
    {
      static bool buttonPressed = 0;
      int buttonPressCount = 0;
      for (int i = 0; i < 50; i++)
      {
        buttonPressCount += !startButton.read();
      }
      if (buttonPressCount == 50)
      {
        buttonPressed = 1;
      }
      if (buttonPressed)
      {
        buttonPressed = 0;
        startButtonPressedFlag = 1;
      }
      if (startButtonPressedFlag && initialButtonPressToken)
      {
        initialButtonPressToken = 0;
        robotLocation.sendNext();
      }
    }
    accelAlgorithm.update();
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), IMU.gyro_Yaw(), output);
    driveWheel.apply(output);
    static unsigned long int prevDisplayed = 0;
    if (((LCDtimer.read_ms() - prevDisplayed) > 40) && !initialButtonPressToken) //about 24Hz flash rate
    {
      LCDDriver.clear();
      LCDDriver.home();
      serialLCD.printf("%d %d %d", robotLocation.getXLocationData(), robotLocation.getYLocationData(), robotLocation.getYawStatsData());
      LCDDriver.setCursor(2, 0);
      serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
      prevDisplayed = LCDtimer.read_ms();
    }

    if (robotLocation.checkMovingStats(accelAlgorithm.getStats()) && !initialButtonPressToken)
    {
      wayPointSignature++;
      switch (wayPointSignature)
      {
        case 1:
          holder.release('r');
          holder.release('l');
          break;

        case 2:
          holder.grasp('r');
          holder.grasp('l');
          break;

        case 3:
          holder.release('r');
          holder.release('l');
          LCDDriver.clear();
          serialLCD.printf("sequence done");
          while (1)
          {
            LCDDriver.setCursor(2, 0);
            accelAlgorithm.update();
            accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
            serialLCD.printf("%.1lf %.1lf %.1lf", accelAlgorithm.getCurrentXPosition(), accelAlgorithm.getCurrentYPosition(), IMU.gyro_Yaw());
          }
          break;
        default:
          LCDDriver.clear();
          serialLCD.printf("WayPoint ERROR");
          while (1)
          {
          }
          break;
      }
      robotLocation.sendNext();
    }
  }
#endif //TEST_DRIVE_NEWTYPE

#ifdef MECA_CLASS_DEBUG

  DigitalIn userButton(USER_BUTTON);
  userButton.mode(PullDown);
  while (1)
  {
    if (userButton.read() == 1)
      break;
  }
#ifdef TEST_RojarArm_UpDownLoop
  RojarArm rojarArmRight(PF_7, PF_9); //PF_7, PF_9->pe10
  rojarArmRight.setMaxPWM(0.96);      //0.92
  QEI rojarArmRightEncoder(PG_0, PD_1, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
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
  Peg pegAttacher(PE_12, PE_14, 0.5, 0.5);
  pegAttacher.launch();
  while (1)
  {
    pegAttacher.update();
  }
#endif // TEST_PEGLaunch

#ifdef TEST_HangerMovement
  ClothHang hanger(PB_10, PB_11);
  hanger.setMaxPWM(0.5);
  hanger.setLength(2000);
  QEI clothHangEncoder(PB_8, PB_9, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
  while (1)
  {
    STLinkTerminal.printf("PULSE:%d \tHANG STATS(extend):%d\r\n", clothHangEncoder.getPulses(), hanger.stats());
    hanger.setEncoderPulse(clothHangEncoder.getPulses());
    hanger.update();
    if (hanger.stats())
      break;
  }
  hanger.setLength(0);
  while (1)
  {
    STLinkTerminal.printf("PULSE:%d \tHANG STATS(reduce):%d\r\n", clothHangEncoder.getPulses(), hanger.stats());
    hanger.setEncoderPulse(clothHangEncoder.getPulses());
    hanger.update();
  }
#endif //TEST_HangerMovement

#ifdef TEST_HoldServo
  ClothHold holder(PE_9, PE_11); //right,leftServo
  while (1)
  {
    holder.grasp('r');
    wait(1);
    while (1)
    {
      if (userButton.read() == 1)
        break;
    }
    holder.release('r');
    wait(1);
    while (1)
    {
      if (userButton.read() == 1)
        break;
    }
  }
#endif //TEST_HoldServo

#ifdef TEST_RojarArmUpDownOnce
  RojarArm rojarArmRight(PD_12, PD_13);
  rojarArmRight.setMaxPWM(0.5);
  rojarArmRight.setHeight(1500);
  QEI rojarArmRightEncoder(PB_8, PB_9, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
  while (1)
  {
    STLinkTerminal.printf("PULSE:%d \tARM STATS(extend):%d\r\n", rojarArmRightEncoder.getPulses(), RojarArmRight.stats());
    rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
    rojarArmRight.update();
    if (rojarArmRight.stats())
      break;
  }
  rojarArmRight.setHeight(0);
  while (1)
  {
    STLinkTerminal.printf("PULSE:%d \tARM STATS(reduce):%d\r\n", rojarArmRightEncoder.getPulses(), RojarArmRight.stats());
    rojarArmRight.setEncoderPulse(rojarArmRightEncoder.getPulses());
    rojarArmRight.update();
  }
#endif //TEST_RojarArmUpDownOnce

#endif //MECA_CLASS_DEBUG
  while (1)
  {
  }
}
