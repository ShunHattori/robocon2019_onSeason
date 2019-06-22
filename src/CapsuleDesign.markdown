# Class design sheet

## FILE TREE
***
- src
  - DriveSource
    - DriveTrain
    - LcationManager
    - MotorDriverAdapter3WD
    - MotorDriverAdapter4WD
    - MWodometry
    - OmniKinematics3WD
    - OmniKinematics4WD
  - MechanismSource
    - ClothHang -done
    - ClothHold
    - Peg
    - RogerArm
    - Servo
  - SensorSource
    - LidarLite
    - MPU9250
    - QEI
  - Others
    - main
    - featureManager
    - NewHavenDisplay

## CLASS ROLES
***
- MechanismSource

機構に関する制御コードを担当,class内でのwhileの使用は禁止し、mainループ内で判定、ループを行う

状態を返すgetterを必ず実装する!!

- SensorSource

センサに関する制御コードを担当,getterで値取得可能

- DriveSource

足回り処理に関する制御コードを担当,統括クラスには位置状態を返すgetterを実装する。
