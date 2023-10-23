package frc.robot.subsystems

import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.advantagekitextensions.loggedwrappers.LoggedIMU
import frc.chargers.hardware.sensors.IMUSim
import frc.chargers.hardware.sensors.NavX

val Gyro by lazy{
    LoggedIMU(
        if (RobotBase.isReal()){
            NavX()
        }else{
            IMUSim(PoseEstimator)
        }
    )
}