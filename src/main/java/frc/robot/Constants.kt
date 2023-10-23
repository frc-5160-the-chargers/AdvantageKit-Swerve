// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.volts
import frc.chargers.constants.MK4i
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.feedforward.Gravity
import frc.chargers.controls.feedforward.LinearMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.swerve.SwerveControl


const val IS_REPLAY: Boolean = false

object DriveConstants{

    private val simTurnFF = LinearMotorFF(0.11.volts,0.26,0.0, distanceUnit = meters, gravity = Gravity.None)
        .convertToAngular(MK4i.TURN_GEAR_RATIO,4.inches)
    val simControlScheme = SwerveControl.PIDSecondOrder(
        turnPIDConstants = PIDConstants(40.0,0.0,0.1),
        turnFF = simTurnFF,
        drivePIDConstants = PIDConstants(0.1,0.0,0.0),
        driveFF = AngularMotorFF(0.11697.volts,0.133420,0.0, angleUnit = radians),
    )

    val realControlScheme = SwerveControl.PIDFirstOrder(
        turnPIDConstants = PIDConstants(40.0,0.0,0.1),
        drivePIDConstants = PIDConstants(0.1,0.0,0.0),
        driveFF = AngularMotorFF(0.11697.volts,0.133420,0.0, angleUnit = radians),
    )

    val trackWidth = 29.inches
    val wheelBase = 29.inches
}





