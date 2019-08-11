#include "DriveSource\DriveTrain.h"
#include "DriveSource\LocationManager.h"
#include "DriveSource\MWodometry.h"
#include "MechanismSource\ClothHang.h"
#include "MechanismSource\ClothHold.h"
#include "MechanismSource\Peg.h"
#include "MechanismSource\RogerArm.h"
#include "MechanismSource\Servo.h"
#include "NewHavenDisplay.h"
#include "SensorSource\DebounceSwitch.h"
#include "SensorSource\MPU9250.h"
#include "SensorSource\QEI.h"
#include "mbed.h"

/*
    使用中の足回りドライブの選択
        USING_4WD➡4輪メカナム・オムニ用逆運動学計算プログラム
        USING_3WD➡3輪オムニ用逆運動学計算プログラム
    使用するドライブのコメントアウトを消去、未使用側をコメントアウトする。
 */
//#define USING_4WD
#define USING_3WD

/*
    新型足回り制御テスト用コード
 */
//#define TEST_DRIVE_NEWTYPE

/*
    ゲーム用新型足回り制御
 */
#define GAME_DRIVE_NEWTYPE

//#define MECA_CLASS_DEBUG
//#define TEST_rogerArm_UpDownLoop
//#define TEST_PEGLaunch      //tested on 6/20(Thur)
//#define TEST_HangerMovement //tested on 6/20(Thur)
//#define TEST_HoldServo
//#define TEST_RogerArmUpDownOnce

/*
    ENCODER_PPR         :   取り付けエンコーダの分解能
    ENCODER_ATTACHED_WHEEL_RADIUS   :   計測輪の半径
    DISTANCE_BETWEEN_ENCODER_WHEELS :   同軸の計測輪取り付け距離
    PERMIT_ERROR_CIRCLE_RADIUS      :   停止地点の許容誤差判定円の半径
    DECREASE_PWM_CIRCLE_RADIUS      :   減速開始判定円の半径
    ESTIMATE_MAX_PWM                :   MDに出力される想定最大PWM
    ESTIMATE_MIN_PWM                :   MDに出力される想定最小PWM
 */
#define ENCODER_PPR 48
#define ENCODER_ATTACHED_WHEEL_RADIUS_BY_NEXUS_ROBOT 5.0
#define ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA_100_DIA 5.08
#define ENCODER_ATTACHDE_WHEEL_RADIUS_BY_HANGDA_50_DIA 2.54
#define DISTANCE_BETWEEN_ENCODER_WHEELS 72
#define PERMIT_ERROR_CIRCLE_RADIUS 3   // 3.5
#define DECREASE_PWM_CIRCLE_RADIUS 110 //150
#define ESTIMATE_MAX_PWM 0.5           // max:0.7, recommend:0.64
#define ESTIMATE_MIN_PWM 0.19

#ifdef USING_4WD
#include "DriveSource\MotorDriverAdapter4WD.h"
#include "DriveSource\OmniKinematics4WD.h"

OmniKinematics4WD OmniKinematics;
MotorDriverAdapter4WD driveWheel(PB_10, PB_11, PE_12, PE_14, PD_12, PD_13, PE_8, PE_10);
float output[4] = {};
#endif // USING_4WD

#ifdef USING_3WD
#include "DriveSource\MotorDriverAdapter3WD.h"
#include "DriveSource\OmniKinematics3WD.h"

OmniKinematics3WD OmniKinematics;
MotorDriverAdapter3WD driveWheel(PB_11, PB_10, PE_12, PE_14, PD_12, PD_13);
float output[3] = {};
#endif // USING_3WD

Timer TimerForQEI;
MPU9250 IMU;
QEI encoderXAxis(PE_9, PF_13, NC, ENCODER_PPR, &TimerForQEI, QEI::X4_ENCODING);
QEI encoderYAxis(PB_5, PC_7, NC, ENCODER_PPR, &TimerForQEI, QEI::X4_ENCODING);
MWodometry odometryXAxis(encoderXAxis, ENCODER_PPR, ENCODER_ATTACHDE_WHEEL_RADIUS_BY_HANGDA_50_DIA);
MWodometry odometryYAxis(encoderYAxis, ENCODER_PPR, ENCODER_ATTACHDE_WHEEL_RADIUS_BY_HANGDA_50_DIA);
LocationManager<int> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometryXAxis, odometryYAxis, IMU, PERMIT_ERROR_CIRCLE_RADIUS, DECREASE_PWM_CIRCLE_RADIUS);

Serial STLinkTerminal(USBTX, USBRX); //Surfaceのターミナルとの通信用ポート
DigitalOut IMUisReadyLED(LED3);      //IMUセンサキャリブレーション完了表示用LED用デジタル出力

int main(void)
{

#ifdef GAME_DRIVE_NEWTYPE
  Serial serialLCD(PC_6, NC, 9600);
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
  Peg pegAttacher(PC_9, PC_8, 0.45, 0.6); //pin ,pin pwm, time
  ClothHang hanger(PF_8, PA_0);           //PF_8, PA_0
  hanger.setMaxPWM(0.6);                  //0.85
  QEI clothHangEncoder(PE_2, PD_11, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
  RogerArm rogerArmRight(PF_7, PF_9); //PF_7, PF_9->pe10
  rogerArmRight.setMaxPWM(0.96);      //0.92
  QEI encoderRogerArmRight(PG_0, PD_1, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
  int wayPointSignature = 1;
  STLinkTerminal.baud(9600);
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
  robotLocation.addPoint(340, -565, 0);
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
    rogerArmRight.setEncoderPulse(encoderRogerArmRight.getPulses());
    rogerArmRight.update();
    pegAttacher.update();
    hanger.setEncoderPulse(clothHangEncoder.getPulses());
    hanger.update();
    accelAlgorithm.update();
    accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
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
          rogerArmRight.setHeight(2600); //2100-洗濯物干し最適高さ , 2850-洗濯バサミ最適高さ
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
          if (rogerArmRight.stats() && armPhase == 1)     //ロジャーアーム展開完了
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
              rogerArmRight.setHeight(3200);
              rogerArmRight.update();
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
          if (rogerArmRight.stats() && armPhase == 2)
          {
            if (hanger.stats() && hangerHasDoneFlag) //洗濯物が竿にかかっているだけの状態
            {
              static unsigned int seq = 1, timerStartFlag = 1;
              if (timerStartFlag)
              {
                clothHangerTimer.start();
                timerStartFlag = 0;
              }
              if (0 < clothHangerTimer.read_ms() && clothHangerTimer.read_ms() < 350 && seq == 1) //`hutatume 1700ms
              {
                pegAttacher.launch();
                seq = 2;
              }
              if (350 < clothHangerTimer.read_ms() && seq == 2)
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
            rogerArmRight.setHeight(2420);
            rogerArmRight.update();
            initialFlag = 0;
          }
          if (rogerArmRight.stats())
          {
            wayPointSignature++;
          }
          break;
        case 5:
          holder.release('l');
          rogerArmRight.setHeight(0); //ロジャーアーム縮小
          //pegAttacher.reload();
          accelAlgorithm.setAllocateErrorCircleRadius(30);
          robotLocation.sendNext();
          wayPointSignature++;
          break;

        case 6:
          accelAlgorithm.setAllocateErrorCircleRadius(3.7);
          holder.grasp('r');
          holder.grasp('l');
          robotLocation.sendNext(); //三本目のポールの手前まで移動
          wayPointSignature++;
          break;

        case 7:
          if (rogerArmRight.stats())
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
    OmniKinematics.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
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
#ifdef TEST_rogerArm_UpDownLoop
  RogerArm rogerArmRight(PF_7, PF_9); //PF_7, PF_9->pe10
  rogerArmRight.setMaxPWM(0.96);      //0.92
  QEI encoderRogerArmRight(PG_0, PD_1, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
  while (1)
  {
    rogerArmRight.setEncoderPulse(encoderRogerArmRight.getPulses());
    rogerArmRight.update();
    static int armPhase = 0;
    if (rogerArmRight.stats() && armPhase == 0)
    {
      wait(2);
      armPhase++;
      rogerArmRight.setHeight(3200);
      rogerArmRight.update();
    }
    else if (rogerArmRight.stats() && armPhase == 1)
    {
      wait(2);
      armPhase = 0;
      rogerArmRight.setHeight(0);
      rogerArmRight.update();
    }
  }
#endif //TEST_rogerArm_UpDownLoop

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

#ifdef TEST_RogerArmUpDownOnce
  RogerArm rogerArmRight(PD_12, PD_13);
  rogerArmRight.setMaxPWM(0.5);
  rogerArmRight.setHeight(1500);
  QEI encoderRogerArmRight(PB_8, PB_9, NC, 48, &TimerForQEI, QEI::X4_ENCODING);
  while (1)
  {
    STLinkTerminal.printf("PULSE:%d \tARM STATS(extend):%d\r\n", encoderRogerArmRight.getPulses(), rogerArmRight.stats());
    rogerArmRight.setEncoderPulse(encoderRogerArmRight.getPulses());
    rogerArmRight.update();
    if (rogerArmRight.stats())
      break;
  }
  rogerArmRight.setHeight(0);
  while (1)
  {
    STLinkTerminal.printf("PULSE:%d \tARM STATS(reduce):%d\r\n", encoderRogerArmRight.getPulses(), rogerArmRight.stats());
    rogerArmRight.setEncoderPulse(encoderRogerArmRight.getPulses());
    rogerArmRight.update();
  }
#endif //TEST_RogerArmUpDownOnce

#endif //MECA_CLASS_DEBUG
  while (1)
  {
  }
}
