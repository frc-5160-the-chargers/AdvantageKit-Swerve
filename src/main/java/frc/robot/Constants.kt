// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.units.*
import frc.chargers.constants.drivetrain.SwerveConstants
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.RobotConfig
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveControl
import frc.chargers.hardware.subsystemutils.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystemutils.swervedrive.swerveCANcoders





const val RESET_POSE_ON_STARTUP: Boolean = true

val CONFIG = RobotConfig(
    isReplay = false,
    tuningMode = true,
    onError = { println("Yay! OnError works!") }
)

object DriveHardware{
    val turnMotors = sparkMaxSwerveMotors(
        topLeft = neoSparkMax(29),
        topRight = neoSparkMax(31),
        bottomLeft = neoSparkMax(22),
        bottomRight = neoSparkMax(5)
    )

    // TBD
    val encoders = swerveCANcoders(
        topLeft = ChargerCANcoder(0),
        topRight = ChargerCANcoder(0),
        bottomLeft = ChargerCANcoder(0),
        bottomRight = ChargerCANcoder(0),
        useAbsoluteSensor = true
    )

    val driveMotors = sparkMaxSwerveMotors(
        topLeft = neoSparkMax(10),
        topRight = neoSparkMax(16),
        bottomLeft = neoSparkMax(30),
        bottomRight = neoSparkMax(3)
    )
}


val DRIVE_CONSTANTS = SwerveConstants.mk4iL2(
    trackWidth = 32.5.inches,
    wheelBase = 32.5.inches
)

val SIM_CONTROL_SCHEME = SwerveControl.PIDSecondOrder(
    turnPIDConstants = PIDConstants(23.0,0.0,0.0),
    turnFF = AngularMotorFF(0.11.volts,0.10,0.0, angleUnit = radians),
    drivePIDConstants = PIDConstants(0.1,0.0,0.0),
    driveFF = AngularMotorFF(0.00162.volts,0.13394,0.0, angleUnit = radians),
)

val REAL_CONTROL_SCHEME = SwerveControl.PIDFirstOrder(
    turnPIDConstants = PIDConstants(10.0,0.0,0.0),
    drivePIDConstants = PIDConstants(0.1,0.0,0.0),
    driveFF = AngularMotorFF(0.11697.volts,0.133420,0.0, angleUnit = radians),
)






