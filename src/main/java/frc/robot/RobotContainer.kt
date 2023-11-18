// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.advantagekitextensions.loggedwrappers.withLogging
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.runPathPlannerAuto
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.controls.pid.PIDConstants
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
import frc.chargers.utils.PathConstraints
import frc.chargers.utils.PathData
import frc.robot.commands.zeroPose
import org.littletonrobotics.junction.Logger
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.subsystems.DRIVE_CONSTANTS
import frc.robot.hardware.subsystems.DRIVE_REAL_CONTROL_SCHEME
import frc.robot.hardware.subsystems.DRIVE_SIM_CONTROL_SCHEME

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [ROBOT]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer: ChargerRobotContainer() {



    
    private val drivetrain: EncoderHolonomicDrivetrain
    private val gyro: IMU




    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {

        DriverController



        if (RobotBase.isReal()){
            val navX = NavX()
            gyro = navX.withLogging()
            ChargerRobot.addToPeriodicLoop{
                Logger.getInstance().recordOutput("NavXDirectHeadingDeg",navX.gyroscope.heading.inUnit(degrees))
            }
            drivetrain = realEncoderHolonomicDrivetrain(
                turnMotors = sparkMaxSwerveMotors(
                    topLeft = neoSparkMax(29),
                    topRight = neoSparkMax(31),
                    bottomLeft = neoSparkMax(22),
                    bottomRight = neoSparkMax(4)
                ),
                turnEncoders = swerveCANcoders(
                    topLeft = ChargerCANcoder(44),
                    topRight = ChargerCANcoder(42),
                    bottomLeft = ChargerCANcoder(43),
                    bottomRight = ChargerCANcoder(45),
                    useAbsoluteSensor = true
                ).withOffsets(
                    topLeftZero = 160.14.degrees,
                    topRightZero = 117.42.degrees,
                    bottomLeftZero = 77.6.degrees,
                    bottomRightZero = (-21.973).degrees
                ),
                driveMotors = sparkMaxSwerveMotors(
                    topLeft = neoSparkMax(10),
                    topRight = neoSparkMax(16),
                    bottomLeft = neoSparkMax(30){inverted = true},
                    bottomRight = neoSparkMax(3)
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





        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)



        DriverController.headingZeroButton.onTrue(InstantCommand(gyro::zeroHeading))


        println("tuning mode: " + DashboardTuner.tuningMode)

        drivetrain.defaultCommand = buildCommand(
            name = "DrivetrainDefaultCommand",
            logIndividualCommands = true
        ){
            +drivetrain.zeroPose()

            loopForever(drivetrain){
                drivetrain.swerveDrive(
                    DriverController.swerveOutput(gyro.heading)
                )
            }

            onEnd(drivetrain::stop)

        }











    }



    override fun testInit(){
        ConsoleLogger.print()
    }


    override val autonomousCommand: Command
        get() = buildCommand(
            name = "FF drivetrain characterization",
            logIndividualCommands = true
        ){

            with(
                PathData(
                    PIDConstants(0.3,0.0,0.0),
                    PIDConstants(0.5,0.0,0.0),
                    PathConstraints(drivetrain.maxLinearVelocity,Acceleration(3.0))
                )
            ){
                drivetrain.runPathPlannerAuto(
                    pathGroupName = "threepiece_auto"
                ){
                    "outtake" mapsTo InstantCommand{println("outtake activated!")}
                    "intake" mapsTo InstantCommand{println("intake activated!")}
                }
            }

            runOnce{
                println("Alliance color: " + DriverStation.getAlliance())
            }

            onEnd(drivetrain::stop)


        }
}
