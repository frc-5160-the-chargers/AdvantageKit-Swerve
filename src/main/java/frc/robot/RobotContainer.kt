// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.*
import frc.chargers.advantagekitextensions.loggedwrappers.withLogging
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.framework.ConsoleLogger
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.IMU
import frc.chargers.hardware.sensors.IMUSim
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.realEncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.simEncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystemutils.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystemutils.swervedrive.swerveCANcoders
import org.littletonrobotics.junction.Logger
import frc.chargers.commands.*
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.subsystems.*

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [ROBOT]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer: ChargerRobotContainer() {



    // swerve drivetrain; implemented in chargerlib
    private val drivetrain: EncoderHolonomicDrivetrain
    // the IMU type is an interface: designed to hold all the data from an IMU,
    // such as altitude, heading, roll, pitch, yaw, etc.
    // also a chargerlib implementation
    private val gyro: IMU




    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {


        DriverController



        if (RobotBase.isReal()){
            // custom AHRS wrapper that utilizes the units library
            val navX = NavX()
            // creates a LoggedIMU class, which allows for log & replay support
            // for classes that implement the IMU interface.
            // implements the IMU interface itself.
            gyro = navX.withLogging()
            ChargerRobot.addToPeriodicLoop{
                Logger.getInstance().recordOutput("NavXDirectHeadingDeg",navX.gyroscope.heading.inUnit(degrees))
            }

            /*
            Drivetrain instantiation.

            Details:
            A. instead of defining individual swerve modules,
            there are helper classes(SwerveMotors and SwerveEncoders) that hold the motors and encoders nessecary.

            Chargerlib has its own Encoder class, with a bunch of wrappers
            that implement it(for example, ChargerCANcoder extends a CANcoder).

            In addition, it has its own EncoderMotorController class, which is a motor bundled with an encoder.
            The "neoSparkMax" function creates an instance of this custom wrapper
            (which also subclasses the regular CANSparkMax class).

             */
            drivetrain = realEncoderHolonomicDrivetrain(
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
                    topLeftZero = 39.11.degrees,
                    topRightZero = 107.92.degrees,
                    bottomLeftZero = 264.46.degrees,
                    bottomRightZero = 345.93.degrees
                ),
                driveMotors = sparkMaxSwerveMotors(
                    topLeft = neoSparkMax(10){inverted = true},
                    topRight = neoSparkMax(16){inverted = false},
                    bottomLeft = neoSparkMax(30){inverted = false},
                    bottomRight = neoSparkMax(3){inverted = false}
                ),
                controlScheme = DRIVE_REAL_CONTROL_SCHEME,
                constants = DRIVE_CONSTANTS,
                gyro = gyro
            )
            println("robot is real")
        }else{
            drivetrain = simEncoderHolonomicDrivetrain(
                turnGearbox = DCMotor.getNEO(1),
                driveGearbox = DCMotor.getNEO(1),
                controlScheme = DRIVE_SIM_CONTROL_SCHEME,
                constants = DRIVE_CONSTANTS
            )
            gyro = IMUSim(headingProviderImpl = drivetrain).withLogging()
            println("robot is sim")
        }


        //val subsystem = object: SubsystemBase(){}
        //RunCommand{println("I exist! Hello!")}.ignoringDisable(true).schedule()




        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)


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
            }

            onEnd{
                drivetrain.stop()
            }
        }













    }



    override fun testInit(){
        ConsoleLogger.print()
    }





    override val autonomousCommand: Command
        get() = DoNothing()
}
