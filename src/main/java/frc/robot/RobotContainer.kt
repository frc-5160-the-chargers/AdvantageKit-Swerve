// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.loggedwrappers.LoggedIMU
import frc.chargers.commands.*
import frc.chargers.hardware.inputdevices.SwerveDriveController
import frc.chargers.hardware.sensors.IMUSim
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.posemonitors.SwervePoseMonitor
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.chargers.wpilibextensions.kinematics.swerve.ModuleStateGroup
import frc.robot.subsystems.getDrivetrain

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {



    val drivetrain = getDrivetrain()
    val poseEstimator = SwervePoseMonitor(drivetrain)
    val gyro = LoggedIMU(
        if (RobotBase.isReal()){
            NavX()
        }else{
            IMUSim(poseEstimator)
        }
    )



    private val controller = SwerveDriveController.fromDefaultBindings(0)



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {

        /*
        loopForever(drivetrain){
                with(gyro as HeadingProvider){
                    drivetrain.swerveDrive(0.5,0.0,-0.2)
                }
            }
         */


        drivetrain.defaultCommand = buildCommand{
            runOnce(drivetrain){
                if (RobotBase.isSimulation()){
                    poseEstimator.resetPose(UnitPose2d())
                }
            }

            loopForever(drivetrain){
                with(gyro as HeadingProvider){
                    drivetrain.swerveDrive(0.5,0.0,0.0)
                }
            }
        }.finallyDo{ drivetrain.stop() }




    }





    val autonomousCommand: Command
        get() = buildCommand{



            loopFor(3.seconds,drivetrain){
                drivetrain.topLeft.setDirectionalPower(0.5,45.degrees)
                drivetrain.topRight.setDirectionalPower(0.5,45.degrees)
                drivetrain.bottomLeft.setDirectionalPower(0.5,45.degrees)
                drivetrain.bottomRight.setDirectionalPower(0.5,45.degrees)
            }



            loopFor(3.seconds,drivetrain){

                drivetrain.currentModuleStates = ModuleStateGroup(
                    Velocity(2.0),Velocity(2.0),Velocity(2.0),Velocity(2.0),
                    45.degrees,45.degrees,45.degrees,45.degrees
                )
                /*
                Drivetrain.topLeft.setDirectionalVelocity(AngularVelocity(0.5),45.degrees)
                Drivetrain.topRight.setDirectionalVelocity(AngularVelocity(0.5),45.degrees)
                Drivetrain.bottomLeft.setDirectionalVelocity(AngularVelocity(0.5),45.degrees)
                Drivetrain.bottomRight.setDirectionalVelocity(AngularVelocity(0.5),45.degrees)

                 */
            }
        }.finallyDo{
            drivetrain.stop()
        }
}
