// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.*
import frc.chargers.hardware.inputdevices.SwerveDriveController
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Gyro
import frc.robot.subsystems.PoseEstimator

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {






    private val controller = SwerveDriveController.fromDefaultBindings(0)



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {

        /*
        loopForever(Drivetrain){
                with(gyro as HeadingProvider){
                    Drivetrain.swerveDrive(0.5,0.0,-0.2)
                }
            }
         */

        Drivetrain
        PoseEstimator
        Gyro


        Drivetrain.defaultCommand = buildCommand(
            Drivetrain,
            name = "DrivetrainDefaultCommand",
            logIndividualCommands = true
        ){

            /*
            runOnce{
                if (RobotBase.isSimulation()){
                    PoseEstimator.resetPose(UnitPose2d())

                }
            }

             */


            loopForever{
                with(Gyro as HeadingProvider){
                    Drivetrain.swerveDrive(0.5,0.0,0.2)
                }
            }


        }.finallyDo{ Drivetrain.stop() }




    }





    val autonomousCommand: Command
        get() = buildCommand(
            name = "Autonomous test command",
            logIndividualCommands = true
        ){

            loopFor(3.seconds,Drivetrain){
                with(Gyro){
                    Drivetrain.velocityDrive(2.0.meters / 1.seconds, Velocity(0.0), 0.degrees/1.seconds)
                }
            }

            loopForever(Drivetrain){
                Drivetrain.stop()
            }

        }
}
