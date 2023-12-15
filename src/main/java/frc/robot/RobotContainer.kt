// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.commands.PPSwerveControllerCommand
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.followPath
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.framework.ConsoleLogger
import frc.chargers.hardware.motorcontrol.rev.SmartCurrentLimit
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.imu.*
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystemutils.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystemutils.swervedrive.swerveCANcoders
import frc.chargers.wpilibextensions.geometry.motion.LinearMotionConstraints
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.constants.*
import frc.robot.hardware.inputdevices.DriverController
import org.littletonrobotics.junction.Logger

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [ROBOT]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer: ChargerRobotContainer() {


    /*
    the IMU type is an interface: designed to hold all the data from an IMU,
    such as altitude, heading, roll, pitch, yaw, etc. Implemented in chargerlib;
    acts as an IO layer in advantagekit due to the NavX and IMUSim classes providing
    log and replay capabilities.
     */
    private val gyroIO: IMU = if (isReal()) NavX() else IMUSim()


    // swerve drivetrain; implemented in chargerlib
    private val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = sparkMaxSwerveMotors(
            topLeft = neoSparkMax(29),
            topRight = neoSparkMax(31),
            bottomLeft = neoSparkMax(22),
            bottomRight = neoSparkMax(4)
        ){
            smartCurrentLimit = SmartCurrentLimit(30.amps)
            voltageCompensationNominalVoltage = 12.volts
        },
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
            topLeft = neoSparkMax(10){ inverted = false },
            topRight = neoSparkMax(16){inverted = true},
            bottomLeft = neoSparkMax(30){inverted = false},
            bottomRight = neoSparkMax(3){inverted = false}
        ){
            smartCurrentLimit = SmartCurrentLimit(40.amps)
            voltageCompensationNominalVoltage = 12.volts
        },
        turnGearbox = DCMotor.getNEO(1),
        driveGearbox = DCMotor.getNEO(1),
        controlScheme = if (isReal()) DRIVE_REAL_CONTROL_SCHEME else DRIVE_SIM_CONTROL_SCHEME,
        constants = DRIVE_CONSTANTS,
        gyro = if(isReal()) gyroIO else null
    )



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        IMUSim.setHeadingSource(drivetrain)
        IMUSim.setChassisSpeedsSource { drivetrain.currentSpeeds }


        Logger.getInstance().apply{
            PPSwerveControllerCommand.setLoggingCallbacks(
                {
                    recordOutput("PPTrajectory/StartPoint/poseMeters", it.initialHolonomicPose)
                    recordOutput("PPTrajectory/timeSecs", it.totalTimeSeconds)
                    recordOutput("PPTrajectory/EndPoint/poseMeters",it.endState.poseMeters)
                    recordOutput("PPTrajectory/EndPoint/angularVelocityRadPerSec",it.endState.angularVelocityRadPerSec)
                },
                {
                    recordOutput("PPTrajectory/Follower/currentPose",it)
                },
                {
                    recordOutput("PPTrajectory/Follower/chassisSpeedsSetpoint/vxMPS",it.vxMetersPerSecond)
                    recordOutput("PPTrajectory/Follower/chassisSpeedsSetpoint/vyMPS",it.vyMetersPerSecond)
                    recordOutput("PPTrajectory/Follower/chassisSpeedsSetpoint/omegaRPS",it.omegaRadiansPerSecond)
                },
                {trans, rot ->
                    recordOutput("PPTrajectory/Follower/chassisSpeedsSetpoint/translationError/xMeters",trans.x)
                    recordOutput("PPTrajectory/Follower/chassisSpeedsSetpoint/translationError/yMeters",trans.y)
                    recordOutput("PPTrajectory/Follower/chassisSpeedsSetpoint/rotationErrorRad",rot.radians)
                }
            )
        }

        DriverController
        configureBindings()

        if (DriverStationSim.getAllianceStationId() != AllianceStationID.Blue1){
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)
        }

        println("tuning mode: " + DashboardTuner.tuningMode)

        // uses kotlin property access syntax to replace setDefaultCommand().
        // in addition, uses the buildCommand DSL, which essentially is a sequential command group
        // in the syntax of a domain-specific language.
        drivetrain.defaultCommand = buildCommand(
            name = "DrivetrainDefaultCommand",
            logIndividualCommands = true
        ){
            addRequirements(drivetrain)

            loopForever{
                drivetrain.swerveDrive(DriverController.swerveOutput(gyroIO.heading))
            }

            onEnd{
                drivetrain.stop()
            }
        }

    }

    private fun configureBindings(){
        val resetAimToAngle = InstantCommand{DriverController.targetHeading = null}

        fun targetAngle(heading: Angle) = InstantCommand{DriverController.targetHeading = heading}

        DriverController.apply{
            if (isReal()) {

                headingZeroButton.onTrue(InstantCommand(gyroIO::zeroHeading))
                poseZeroButton.onTrue(
                    InstantCommand{
                        drivetrain.poseEstimator.resetPose(UnitPose2d())
                        println("Pose has been reset.")
                    }
                )
            }

            pointNorthButton.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle)
            pointEastButton.onTrue(targetAngle(90.degrees)).onFalse(resetAimToAngle)
            pointSouthButton.onTrue(targetAngle(180.degrees)).onFalse(resetAimToAngle)
            pointWestButton.onTrue(targetAngle(270.degrees)).onFalse(resetAimToAngle)
        }

    }



    override fun testInit(){
        ConsoleLogger.print()
    }


    override val autonomousCommand: Command
        get() = buildCommand{

            loopForever(drivetrain){
                drivetrain.velocityDrive(Velocity(0.5),Velocity(0.0), AngularVelocity(0.0))
            }

        }
}
