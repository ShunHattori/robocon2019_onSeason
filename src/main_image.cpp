/*
#include "mbed.h"
#include "MWodometry.h"
#include "LocationManager.h"
#include "DriveTrain.h"
#include "NewHavenDisplay.h"
#include "QEI.h"
#include "MPU9250.h"

ENCODER_PULSE_PER_ROUND         :   取り付けエンコーダの分解能
ENCODER_ATTACHED_WHEEL_RADIUS   :   計測輪の半径
DISTANCE_BETWEEN_ENCODER_WHEELS :   同軸の計測輪取り付け距離
PERMIT_ERROR_CIRCLE_RADIUS      :   停止地点の許容誤差判定円の半径
DECREASE_PWM_CIRCLE_RADIUS      :   減速開始判定円の半径
ESTIMATE_MAX_PWM                :   MDに出力される想定最大PWM
ESTIMATE_MIN_PWM                :   MDに出力される想定最小PWM
DRIVETRAIN_UPDATE_CYCLE         :   速度計算アルゴリズムの更新周期(s)

#define ENCODER_PULSE_PER_ROUND 48
#define ENCODER_ATTACHED_WHEEL_RADIUS_BY_NEXUS_ROBOT 5.0
#define ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA 5.08
#define DISTANCE_BETWEEN_ENCODER_WHEELS 72
#define PERMIT_ERROR_CIRCLE_RADIUS 2 //3.5
#define DECREASE_PWM_CIRCLE_RADIUS 120
#define ESTIMATE_MAX_PWM 0.6
#define ESTIMATE_MIN_PWM 0.07
#define DRIVETRAIN_UPDATE_CYCLE 0.15

#include "OmniKinematics3WD.h"
#include "MotorDriverAdapter3WD.h"
OmniKinematics3WD wheel;
MotorDriverAdapter3WD driveWheel(PB_11, PB_10, PE_12, PE_14, PD_12, PD_13);
float output[3] = {};

Timer QEITimer;
MPU9250 IMU;
QEI encoder_XAxis_1(PE_9, PF_13, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X2_ENCODING);
QEI encoder_YAxis_1(PB_5, PC_7, NC, ENCODER_PULSE_PER_ROUND, &QEITimer, QEI::X2_ENCODING);
MWodometry odometry_XAxis_1(encoder_XAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA);
MWodometry odometry_YAxis_1(encoder_YAxis_1, ENCODER_PULSE_PER_ROUND, ENCODER_ATTACHED_WHEEL_RADIUS_BY_HANGFA);
LocationManager<int> robotLocation(0, 0, 0);
DriveTrain accelAlgorithm(robotLocation, odometry_XAxis_1, odometry_YAxis_1, IMU, PERMIT_ERROR_CIRCLE_RADIUS, DECREASE_PWM_CIRCLE_RADIUS);
Ticker updateOutput;

Serial PC(USBTX, USBRX);
DigitalOut shoot1(PG_0); //フリップ用信号ピン
DigitalOut shoot2(PD_1); //フリップ用信号ピン
DigitalOut IMUReady(LED2);

int main()
{
    PC.baud(9600);
    IMU.setup(PB_9, PB_8);
    accelAlgorithm.setMaxOutput(ESTIMATE_MAX_PWM);
    accelAlgorithm.setMinOutput(ESTIMATE_MIN_PWM);
    wheel.setMaxPWM(ESTIMATE_MAX_PWM);
    IMUReady = 1;

    robotLocation.addPoint(0, 135, 0);
    robotLocation.addPoint(100, 135, 0);
    robotLocation.addPoint(0, 120, 0);
    robotLocation.addPoint(450, 120, 0);
    robotLocation.addPoint(500, 500, 0);
    robotLocation.addPoint(0, 0, 0);
    robotLocation.addPoint(0, 200, 0);
    robotLocation.addPoint(0, 500, 0);
    robotLocation.addPoint(0, 0, 0);

    while (1)
    {
        accelAlgorithm.setCurrentYawPosition(IMU.gyro_Yaw());
        accelAlgorithm.update();
        wheel.getOutput(accelAlgorithm.getXVector(), accelAlgorithm.getYVector(), accelAlgorithm.getYawVector(), output);
        driveWheel.apply(output);

        if (accelAlgorithm.robotIsMoving()) //ロボットが移動中だったらtrueになるフラグ    vectorがどんなくらいの大きさになったら。とかで制御すると思う
        {
            if (LocationManager.getCurrentPoint() == 1) //最後に移動したポイントの番号を返すメソッドを作る　robotIsMovingと併用で次のポイントの移動中に実行するタスクを指定できるようにする
            {
            }
            else if (LocationManager.getCurrentPoint() == 2)
            {
            }
            else if (LocationManager.getCurrentPoint() == 3)
            {
            }
        }
        else
        {
            if (LocationManager.getCurrentPoint() == 1)
            {
                robotLocation.sendNext();
            }
            if (LocationManager.getCurrentPoint() == 2)
            {
                robotLocation.sendNext();
            }
            if (LocationManager.getCurrentPoint() == 3)
            {
                robotLocation.sendNext();
            }
        }
    }
}
*/