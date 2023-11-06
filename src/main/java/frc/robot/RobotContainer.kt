// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.*
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.inputdevices.SwerveDriveController
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Gyro

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [ROBOT]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer: ChargerRobotContainer() {






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


        Drivetrain
        Gyro



        println("tuning mode: " + DashboardTuner.tuningMode)


        Drivetrain.defaultCommand = buildCommand(
            name = "DrivetrainDefaultCommand",
            logIndividualCommands = true
        ){
            runOnce(Drivetrain){
                if (RobotBase.isSimulation() && RESET_POSE_ON_STARTUP){
                    Drivetrain.poseEstimator.resetPose(UnitPose2d())
                }
            }


            loopForever(Drivetrain){
                Drivetrain.swerveDrive(controller.swerveOutput)
            }

            loopForever(Drivetrain){
                Drivetrain.stop()
            }


        }.finallyDo{ Drivetrain.stop() }






    }





    override val autonomousCommand: Command
        get() = buildCommand(
            name = "FF drivetrain characterization",
            logIndividualCommands = true
        ){
            runOnce(Drivetrain){
                if (RobotBase.isSimulation() && RESET_POSE_ON_STARTUP){
                    Drivetrain.poseEstimator.resetPose(UnitPose2d())
                }
            }

            +Drivetrain.characterize()
        }
}
