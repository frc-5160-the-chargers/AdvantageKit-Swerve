// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.commands.DoNothing
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.buildCommand
import frc.chargers.constants.tuning.DashboardTuner
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.framework.ConsoleLogger
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.cameras.vision.ApriltagCamSim
import frc.chargers.hardware.sensors.cameras.vision.VisionPipeline
import frc.chargers.hardware.sensors.cameras.vision.VisionResult
import frc.chargers.hardware.sensors.cameras.vision.advantageKitApriltagPipeline
import frc.chargers.hardware.sensors.cameras.vision.limelight.Limelight
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.imu.IMU
import frc.chargers.hardware.sensors.imu.IMUSim
import frc.chargers.hardware.sensors.imu.NavX
import frc.chargers.hardware.sensors.imu.advantageKitIMU
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystemutils.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystemutils.swervedrive.swerveCANcoders
import frc.chargers.wpilibextensions.geometry.rotation.Rotation3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTranslation3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.robot.hardware.inputdevices.DriverController

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [ROBOT]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer: ChargerRobotContainer() {

    private object RealHardware{
        val navX = NavX()

        val limelight = Limelight(
            lensHeight = 0.5.meters,
            mountAngle = 30.degrees
        )

        val swerveTurnMotors = sparkMaxSwerveMotors(
            topLeft = neoSparkMax(29), /*inverted = false*/
            topRight = neoSparkMax(31) /*{inverted = true}*/ ,
            bottomLeft = neoSparkMax(22), /*inverted = false*/
            bottomRight = neoSparkMax(4) /*inverted = false*/
        )

        val swerveDriveMotors = sparkMaxSwerveMotors(
            topLeft = neoSparkMax(10){inverted = true},
            topRight = neoSparkMax(16){inverted = false},
            bottomLeft = neoSparkMax(30){inverted = false},
            bottomRight = neoSparkMax(3){inverted = false}
        )

        val swerveEncoders = swerveCANcoders(
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
        )
    }


    /*
    the IMU type is an interface: designed to hold all the data from an IMU,
    such as altitude, heading, roll, pitch, yaw, etc.

    the function "advantageKitIMU" creates a
     */
    private val gyro: IMU = advantageKitIMU(
        "IMU",
        // implementation of the IMU interface on the real robot
        realImpl = RealHardware.navX,
        // implementation of the IMU interface on the sim robot.
        simImpl = IMUSim()
    )

    // swerve drivetrain; implemented in chargerlib
    private val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = RealHardware.swerveTurnMotors,
        turnEncoders = RealHardware.swerveEncoders,
        driveMotors = RealHardware.swerveDriveMotors,
        turnGearbox = DCMotor.getNEO(1),
        driveGearbox = DCMotor.getNEO(1),
        controlScheme = if (RobotBase.isReal()) DRIVE_REAL_CONTROL_SCHEME else DRIVE_SIM_CONTROL_SCHEME,
        constants = DRIVE_CONSTANTS,
        gyro = if(RobotBase.isReal()) gyro else null
    ).also{
        IMUSim.setHeadingSource(it)
        IMUSim.setChassisSpeedsSource { it.currentSpeeds }
    }


    /*
    Creates an instance of the VisionPipeline<VisionResult.Apriltag> interface,
    which is essentially a generic vision camera that can detect apriltags.

    The "advantageKitApriltagPipeline" function creates an implementation of this interface
    with advantagekit support: I.E logging & replay.
     */
    private val visionCamera: VisionPipeline<VisionResult.Apriltag> =
        advantageKitApriltagPipeline(
            "ApriltagVision",
            // implementation of VisionPipeline<VisionResult.Apriltag> used on real robot
            realImpl = RealHardware.limelight.ApriltagPipeline(id = 0),
            // implementation of VisionPipeline<VisionResult.Apriltag> used in simulation
            simImpl = ApriltagCamSim(
                camName = "Sim Camera",
                robotPoseSupplier = drivetrain.poseEstimator,
                robotToCam = UnitTransform3d(
                    UnitTranslation3d(
                        0.0.inches,
                        0.inches,
                        0.5.meters
                    ),
                    Rotation3d(
                        0.degrees,
                        60.degrees,
                        0.degrees
                    )
                ),
                fov = 180.degrees,
                ledRange = 20.meters,
                minTargetArea = 10.0,
                cameraResHeight = 640,
                cameraResWidth = 480,
                fieldMap = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField()
            ).also{
                if (RobotBase.isSimulation()){
                    drivetrain.poseEstimator.addPoseSuppliers(it.PoseEstimator())
                }
            }
        )







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
        get() = DoNothing()
}
