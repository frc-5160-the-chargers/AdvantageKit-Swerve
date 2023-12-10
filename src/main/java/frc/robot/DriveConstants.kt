// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.*
import frc.chargers.constants.drivetrain.SwerveConstants
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveControl
import frc.chargers.utils.Precision



const val AIM_TO_TARGET_ENABLED = true

val DRIVE_CONSTANTS = SwerveConstants.mk4iL2(
    maxModuleSpeed = 4.55.meters / 1.seconds,
    trackWidth = 32.5.inches,
    wheelBase = 32.5.inches
)

val DRIVE_SIM_CONTROL_SCHEME = SwerveControl.PIDFirstOrder(
    turnPIDConstants = PIDConstants(15.0,0.0,0.0),
    //turnConstraints = AngularMotionConstraints(5.0.rotations / 1.seconds,AngularAcceleration(7.0)),
    //turnFF = AngularMotorFF(0.00126.volts,0.4250,0.0, angleUnit = radians),
    drivePIDConstants = PIDConstants(0.1,0.0,0.0),
    driveFF = AngularMotorFF(0.00162.volts,0.13394,0.0, angleUnit = radians),
)

val DRIVE_REAL_CONTROL_SCHEME = SwerveControl.PIDFirstOrder(
    turnPIDConstants = PIDConstants(3.0,0.0,0.0),
    turnPrecision = Precision.Within(0.2.degrees),
    drivePIDConstants = PIDConstants(0.1,0.0,0.0),
    driveFF = AngularMotorFF(0.12117.volts,0.13210,0.0, angleUnit = radians),
)






