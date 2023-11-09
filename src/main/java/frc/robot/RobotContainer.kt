// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.loggedwrappers.LoggedIMU
import frc.chargers.commands.*
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.inputdevices.SwerveDriveController
import frc.chargers.hardware.sensors.IMU
import frc.chargers.hardware.sensors.IMUSim
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.realEncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.simEncoderHolonomicDrivetrain
import frc.robot.commands.zeroPose

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [ROBOT]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer: ChargerRobotContainer() {



    
    private val drivetrain: EncoderHolonomicDrivetrain
    private val gyro: IMU



    private val controller = SwerveDriveController.fromDefaultBindings(
        port = 1,
        driveMultiplier = 1.0,
        rotationMultiplier = -1.0,
        turboModeMultiplierRange = 1.0..2.0,
        precisionModeDividerRange = 1.0..4.0,
        deadband = 0.2,
    )



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {


        
        
        if (RobotBase.isReal()){
            gyro = LoggedIMU(NavX())
            drivetrain = realEncoderHolonomicDrivetrain(
                turnMotors = DriveHardware.turnMotors,
                turnEncoders = DriveHardware.encoders,
                driveMotors = DriveHardware.driveMotors,
                controlScheme = REAL_CONTROL_SCHEME,
                constants = DRIVE_CONSTANTS,
                gyro = gyro
            )
        }else{
            drivetrain = simEncoderHolonomicDrivetrain(
                turnGearbox = DCMotor.getNEO(1),
                driveGearbox = DCMotor.getNEO(1),
                controlScheme = SIM_CONTROL_SCHEME,
                constants = DRIVE_CONSTANTS
            )
            gyro = LoggedIMU(IMUSim(headingProviderImpl = drivetrain))
        }

        



        println("tuning mode: " + DashboardTuner.tuningMode)



        drivetrain.defaultCommand = buildCommand(
            name = "DrivetrainDefaultCommand",
            logIndividualCommands = true
        ){
            +drivetrain.zeroPose()

            loopForever(drivetrain){
                drivetrain.swerveDrive(0.5,0.0,0.3)
            }

        }.finallyDo{ drivetrain.stop() }








    }





    override val autonomousCommand: Command
        get() = buildCommand(
            name = "FF drivetrain characterization",
            logIndividualCommands = true
        ){
            +drivetrain.zeroPose()

            +drivetrain.characterizeTurnMotors()

        }
}
