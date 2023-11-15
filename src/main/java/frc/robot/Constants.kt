// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.constants.drivetrain.SwerveConstants
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.RobotConfig
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.subsystemutils.swervedrive.SwerveControl
import frc.chargers.hardware.subsystemutils.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystemutils.swervedrive.swerveCANcoders
import frc.chargers.utils.Precision


val RESET_POSE_ON_STARTUP = RobotBase.isSimulation()

val CONFIG = RobotConfig(
    isReplay = false,
    tuningMode = false,
    onError = { println("An error has occurred. Normally, this will write to the crash tracker disc. ") }
)

object DriveHardware{
    val turnMotors = sparkMaxSwerveMotors(
        topLeft = neoSparkMax(29),
        topRight = neoSparkMax(31),
        bottomLeft = neoSparkMax(22),
        bottomRight = neoSparkMax(3)
    ){
        idleMode = CANSparkMax.IdleMode.kBrake
        //inverted = true
    }


    val encoders = swerveCANcoders(
        topLeft = ChargerCANcoder(44){ magnetOffset = -233.34.degrees },
        topRight = ChargerCANcoder(42){ magnetOffset = -166.7.degrees},
        bottomLeft = ChargerCANcoder(43){magnetOffset = -187.degrees},
        bottomRight = ChargerCANcoder(45){magnetOffset = -190.98.degrees},
        useAbsoluteSensor = true
    ){
        sensorDirection = SensorDirectionValue.Clockwise_Positive
    }



    val driveMotors = sparkMaxSwerveMotors(
        topLeft = neoSparkMax(10),
        topRight = neoSparkMax(16),
        bottomLeft = neoSparkMax(30),
        bottomRight = neoSparkMax(4)
    ){
        idleMode = CANSparkMax.IdleMode.kBrake
    }
}


val DRIVE_CONSTANTS = SwerveConstants.mk4iL2(
    trackWidth = 32.5.inches,
    wheelBase = 32.5.inches
)

val SIM_CONTROL_SCHEME = SwerveControl.PIDSecondOrder(
    turnPIDConstants = PIDConstants(15.0,0.0,0.0),
    turnFF = AngularMotorFF(0.00126.volts,0.34,0.0, angleUnit = radians),
    drivePIDConstants = PIDConstants(0.1,0.0,0.0),
    driveFF = AngularMotorFF(0.00162.volts,0.13394,0.0, angleUnit = radians),
)

val REAL_CONTROL_SCHEME = SwerveControl.PIDFirstOrder(
    turnPIDConstants = PIDConstants(4.0,0.0,0.0),
    // turnPrecision = Precision.Within(1.degrees),
    drivePIDConstants = PIDConstants(0.1,0.0,0.0),
    driveFF = AngularMotorFF(0.11697.volts,0.133420,0.0, angleUnit = radians),
)






