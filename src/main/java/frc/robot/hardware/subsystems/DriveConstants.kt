// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.hardware.subsystems

import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.constants.drivetrain.SwerveConstants
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveControl
import frc.chargers.utils.Precision


val RESET_POSE_ON_STARTUP = RobotBase.isSimulation()


val DRIVE_CONSTANTS = SwerveConstants.mk4iL2(
    maxModuleSpeed = Velocity(4.0),
    trackWidth = 32.5.inches,
    wheelBase = 32.5.inches
)

val DRIVE_SIM_CONTROL_SCHEME = SwerveControl.PIDSecondOrder(
    turnPIDConstants = PIDConstants(15.0,0.0,0.0),
    turnFF = AngularMotorFF(0.00126.volts,0.34,0.0, angleUnit = radians),
    drivePIDConstants = PIDConstants(0.1,0.0,0.0),
    driveFF = AngularMotorFF(0.00162.volts,0.13394,0.0, angleUnit = radians),
)

val DRIVE_REAL_CONTROL_SCHEME = SwerveControl.PIDFirstOrder(
    turnPIDConstants = PIDConstants(3.0,0.0,0.0),
    turnPrecision = Precision.Within(0.5.degrees),
    drivePIDConstants = PIDConstants(0.1,0.0,0.0),
    driveFF = AngularMotorFF(0.11697.volts,0.133420,0.0, angleUnit = radians),
)






