// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Velocity
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.commands.DoNothing
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.buildCommand
import frc.chargers.commands.characterizeDriveMotors
import frc.chargers.commands.drivetrainCommands.followPath
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.framework.ConsoleLogger
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.imu.*
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystemutils.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystemutils.swervedrive.swerveCANcoders
import frc.chargers.wpilibextensions.geometry.motion.LinearMotionConstraints
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.hardware.inputdevices.DriverController

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [ROBOT]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer: ChargerRobotContainer() {




    /*
    the IMU type is an interface: designed to hold all the data from an IMU,
    such as altitude, heading, roll, pitch, yaw, etc.

    the function "advantageKitIMU" creates a
     */
    private val gyro: IMU = if (RobotBase.isReal()) NavX() else IMUSim()

    // swerve drivetrain; implemented in chargerlib
    private val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = sparkMaxSwerveMotors(
            topLeft = neoSparkMax(29), /*inverted = false*/
            topRight = neoSparkMax(31) /*{inverted = true}*/ ,
            bottomLeft = neoSparkMax(22), /*inverted = false*/
            bottomRight = neoSparkMax(4) /*inverted = false*/
        ),
        turnEncoders = swerveCANcoders(
            topLeft = ChargerCANcoder(44),
            topRight = ChargerCANcoder(42),
            bottomLeft = ChargerCANcoder(43),
            bottomRight = ChargerCANcoder(45),
            useAbsoluteSensor = true
        ).withOffsets(
            topLeftZero = 0.602.radians,
            topRightZero = 1.81.radians,
            bottomLeftZero = 1.48.radians,
            bottomRightZero = 2.936.radians
        ),
        driveMotors = sparkMaxSwerveMotors(
            topLeft = neoSparkMax(10){inverted = false},
            topRight = neoSparkMax(16){inverted = true},
            bottomLeft = neoSparkMax(30){inverted = false},
            bottomRight = neoSparkMax(3){inverted = false}
        ).also{
            println("topLeft inverted: " + it.topLeft.inverted)
            println("topRight inverted: " + it.topRight.inverted)
            println("bottomLeft inverted: " + it.bottomLeft.inverted)
            println("bottomRight inverted: " + it.bottomRight.inverted)
        },
        turnGearbox = DCMotor.getNEO(1),
        driveGearbox = DCMotor.getNEO(1),
        controlScheme = if (RobotBase.isReal()) DRIVE_REAL_CONTROL_SCHEME else DRIVE_SIM_CONTROL_SCHEME,
        constants = DRIVE_CONSTANTS,
        gyro = if(RobotBase.isReal()) gyro else HeadingProvider{Angle(0.0)}
    ).also{
        IMUSim.setHeadingSource(it)
        IMUSim.setChassisSpeedsSource { it.currentSpeeds }
    }



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {

        configureBindings()



        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)
        println("tuning mode: " + DashboardTuner.tuningMode)

        // uses kotlin property access syntax to replace setDefaultCommand().
        // in addition, uses the buildCommand DSL, which essentially is a sequential command group
        // in the syntax of a domain-specific language.
        drivetrain.defaultCommand = buildCommand(
            name = "DrivetrainDefaultCommand",
            logIndividualCommands = true
        ){
            addRequirements(drivetrain)

            runOnce{
                if (RESET_POSE_ON_STARTUP) drivetrain.poseEstimator.resetPose(UnitPose2d())
            }

            loopForever{
                drivetrain.swerveDrive(DriverController.swerveOutput(gyro.heading), fieldRelative = true)
                //drivetrain.velocityDrive(Velocity(1.0),Velocity(0.0), AngularVelocity(1.0))
            }

            onEnd{
                drivetrain.stop()
            }
        }

    }

    private fun configureBindings(){
        DriverController

        val resetAimToAngle = InstantCommand{DriverController.targetHeading = null}

        DriverController.headingZeroButton.onTrue(InstantCommand(gyro::zeroHeading))
        DriverController.pointNorthButton.onTrue(
            InstantCommand{DriverController.targetHeading = 0.degrees}
        ).onFalse(resetAimToAngle)
        DriverController.pointEastButton.onTrue(
            InstantCommand{DriverController.targetHeading = 90.degrees}
        ).onFalse(resetAimToAngle)
        DriverController.pointSouthButton.onTrue(
            InstantCommand{DriverController.targetHeading = 180.degrees}
        ).onFalse(resetAimToAngle)
        DriverController.pointWestButton.onTrue(
            InstantCommand{DriverController.targetHeading = 270.degrees}
        ).onFalse(resetAimToAngle)
    }



    override fun testInit(){
        ConsoleLogger.print()
    }


    override val autonomousCommand: Command
        get() = buildCommand{
            //+drivetrain.characterizeDriveMotors(forwards = false)

            drivetrain.followPath(
                trajectoryName = "minipath",
                translationConstants = PIDConstants(0.2,0.0,0.0),
                rotationConstants = PIDConstants(0.3,0.0,0.0),
                pathConstraints = LinearMotionConstraints(Velocity(0.5), Acceleration(0.5)),
                isFirstPath = true
            )


        }
}
