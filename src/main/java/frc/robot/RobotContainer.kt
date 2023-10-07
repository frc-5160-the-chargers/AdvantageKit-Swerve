// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.commands.PPSwerveControllerCommand
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.commands.*
import frc.chargers.commands.drivetrainCommands.followPath
import frc.chargers.constants.MK4i
import frc.chargers.controls.feedforward.AngularMotorFF
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.inputdevices.SwerveDriveController
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.realEncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.simEncoderHolonomicDrivetrain
import frc.chargers.hardware.swerve.control.SwerveAngleControl
import frc.chargers.hardware.swerve.control.SwerveSpeedControl
import frc.chargers.hardware.swerve.sparkMaxDriveMotors
import frc.chargers.hardware.swerve.sparkMaxTurnMotors
import frc.chargers.hardware.swerve.swerveCANcoders
import frc.chargers.utils.PathConstraints
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile
import org.littletonrobotics.junction.Logger

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {

    val drivetrain: EncoderHolonomicDrivetrain





    /*
    private val turnC: SwerveAngleControl = SwerveAngleControl.ProfiledPID(
        PIDConstants(20.0,0.0,0.0),
        AngularTrapezoidProfile.Constraints(AngularVelocity(6.0),AngularAcceleration(6.0)),
    )
     */







    private val turnC: SwerveAngleControl = SwerveAngleControl.PID(
        PIDConstants(40.0,0.0,0.1)
    )

    private val emptyGyro = object: HeadingProvider{
        override val heading: Angle
            get() = Angle(0.0)
    }


    private val velC: SwerveSpeedControl = SwerveSpeedControl(
        pidConstants = PIDConstants(0.1,0.0,0.0),
        ff = AngularMotorFF(0.11697.volts,0.133420,0.0, angleUnit = radians)
    )

    private val controller = SwerveDriveController.fromDefaultBindings(0)



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the trigger bindings
        if (RobotBase.isReal()){
            drivetrain = realEncoderHolonomicDrivetrain(
                turnMotors = sparkMaxTurnMotors(
                    topLeft = neoSparkMax(0),
                    topRight = neoSparkMax(1),
                    bottomLeft = neoSparkMax(0),
                    bottomRight = neoSparkMax(0)
                ),
                turnEncoders = swerveCANcoders(
                    topLeft = ChargerCANcoder(0),
                    topRight = ChargerCANcoder(0),
                    bottomLeft = ChargerCANcoder(0),
                    bottomRight = ChargerCANcoder(0),
                    useAbsoluteSensor = true
                ),
                driveMotors = sparkMaxDriveMotors(
                    topLeft = neoSparkMax(0),
                    topRight = neoSparkMax(1),
                    bottomLeft = neoSparkMax(0),
                    bottomRight = neoSparkMax(0)
                ),
                turnControl = turnC,
                velocityControl = velC,
                NavX(),
                driveGearRatio = MK4i.GEAR_RATIO_L2,
                turnGearRatio = MK4i.TURN_GEAR_RATIO,
                trackWidth = 29.inches,
                wheelBase = 29.inches,
                wheelDiameter = MK4i.WHEEL_DIAMETER,
                fieldRelativeDrive = false
            )
        }else{
            drivetrain = simEncoderHolonomicDrivetrain(
                turnGearbox = DCMotor.getNEO(1),
                driveGearbox = DCMotor.getNEO(1),
                turnGearRatio = MK4i.TURN_GEAR_RATIO,
                driveGearRatio = MK4i.GEAR_RATIO_L2,
                turnControl = turnC,
                velocityControl = velC,
                trackWidth = 29.inches,
                wheelBase = 29.inches,
                wheelDiameter = MK4i.WHEEL_DIAMETER,
                fieldRelativeDrive = true
            )
        }

        configureBindings()
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger] constructor with an arbitrary
     * predicate, or via the named factories in [ ]'s subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [Flight][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        drivetrain.defaultCommand = RunCommand(drivetrain){
            /*
            drivetrain.apply{
                topLeft.setDirection(45.degrees)
                topRight.setDirection(45.degrees)
                bottomLeft.setDirection(45.degrees)
                bottomRight.setDirection(45.degrees)
                topLeft.setPower(0.5)
                topRight.setPower(0.5)
                bottomLeft.setPower(0.5)
                bottomRight.setPower(0.5)
            }
             */
            drivetrain.swerveDrive(0.0,0.5,0.2)
        }.finallyDo{
            drivetrain.stop()
        }
    }

    val odoTestCommand: Command = buildCommand{
        val kinematics: SwerveDriveKinematics = drivetrain.kinematics
    }

    val autonomousCommand: Command
        get() = buildCommand{

            /*
            loopFor(3.seconds,drivetrain){
                drivetrain.swerveDrive(0.5,0.0,0.2)
            }
             */

            /*
            loopFor(5.seconds,drivetrain){
                drivetrain.topLeft.apply{
                    setDirection(45.degrees)
                    setPower(0.5)
                }
                drivetrain.topRight.apply{
                    setDirection(45.degrees)
                    setPower(0.5)
                }
                drivetrain.bottomLeft.apply{
                    setDirection(315.degrees)
                    setPower(0.5)
                }
                drivetrain.bottomRight.apply{
                    setDirection(315.degrees)
                    setPower(0.5)
                }
            }

             */




            loopFor(3.seconds,drivetrain){
                drivetrain.velocityDrive(Velocity(0.0),Velocity(3.0),0.degrees / 1.seconds)
            }










            /*
            drivetrain.followPath(
                "path_test_2",
                PIDConstants(0.2,0.0,0.0),
                PIDConstants(0.1,0.0,0.0),
                PathConstraints(Velocity(4.5),Acceleration(2.0)),
                isFirstPath = true
            )
             */










            loopForever(drivetrain){
                drivetrain.stop()
            }
        }
}
