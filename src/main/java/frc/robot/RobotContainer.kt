// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.commands.DoNothing
import frc.chargers.commands.setDefaultRunCommand
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
import frc.chargers.hardware.swerve.control.TurnPID
import frc.chargers.hardware.swerve.control.VelocityPID
import frc.chargers.hardware.swerve.sparkMaxDriveMotors
import frc.chargers.hardware.swerve.sparkMaxTurnMotors
import frc.chargers.hardware.swerve.swerveCANcoders
import frc.chargers.wpilibextensions.geometry.AngularTrapezoidProfile
import frc.chargers.wpilibextensions.kinematics.swerve.ModuleSpeeds
import org.littletonrobotics.junction.Logger

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {

    val drivetrain: EncoderHolonomicDrivetrain

    val field = Field2d()


    /*
    private val turnC: TurnPID = TurnPID.Profiled(
        PIDConstants(14.0,0.0,0.1),
        AngularTrapezoidProfile.Constraints(AngularVelocity(6.0),AngularAcceleration(3.5)),
        //AngularMotorFF(0.0.volts,1.0,0.0,angleUnit = radians)
    )
     */


    private val turnC: TurnPID = TurnPID.Basic(
        PIDConstants(20.0,0.0,0.1)
    )

    private val velC: VelocityPID = VelocityPID(
        pidConstants = PIDConstants(2.5,0.0,0.0),
        ff = AngularMotorFF(0.1.volts,0.0,0.0, angleUnit = radians)
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
                    bottomRight = ChargerCANcoder(0)
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
                gearRatio = MK4i.GEAR_RATIO_L2,
                trackWidth = 5.meters,
                wheelBase = 5.meters,
                wheelDiameter = MK4i.WHEEL_DIAMETER,
                fieldRelativeDrive = false
            )
        }else{
            drivetrain = simEncoderHolonomicDrivetrain(
                turnGearbox = DCMotor.getNEO(1),
                driveGearbox = DCMotor.getNEO(1),
                turnGearRatio = MK4i.TURN_GEAR_RATIO,
                driveGearRatio = MK4i.GEAR_RATIO_L2,
                gyro = object: HeadingProvider {
                    override val heading: Angle = Angle(0.0)
                },
                turnControl = turnC,
                velocityControl = velC,
                trackWidth = 5.meters,
                wheelBase = 5.meters,
                wheelDiameter = MK4i.WHEEL_DIAMETER,
                fieldRelativeDrive = false
            )
        }

        Logger.getInstance().recordOutput("TopLeftSwerveModule/postOptimizeDesiredDirection", 0.0)
        Logger.getInstance().recordOutput("TopRightSwerveModule/postOptimizeDesiredDirection", 0.0)
        Logger.getInstance().recordOutput("BottomLeftSwerveModule/postOptimizeDesiredDirection", 0.0)
        Logger.getInstance().recordOutput("BottomRightSwerveModule/postOptimizeDesiredDirection", 0.0)

        Logger.getInstance().recordOutput("TopLeftSwerveModule/initialDesiredDirection", 0.0)
        Logger.getInstance().recordOutput("TopRightSwerveModule/initialDesiredDirection", 0.0)
        Logger.getInstance().recordOutput("BottomLeftSwerveModule/initialDesiredDirection", 0.0)
        Logger.getInstance().recordOutput("BottomRightSwerveModule/initialDesiredDirection", 0.0)

        Logger.getInstance().recordOutput("TopLeftSwerveModule/angleDelta", 0.0)
        Logger.getInstance().recordOutput("TopRightSwerveModule/angleDelta", 0.0)
        Logger.getInstance().recordOutput("BottomLeftSwerveModule/angleDelta", 0.0)
        Logger.getInstance().recordOutput("BottomRightSwerveModule/angleDelta", 0.0)
        
        

        // println(drivetrain.maxRotationalVelocity.inUnit(degrees/ seconds))


        configureBindings()

        SmartDashboard.putData("Field",field)
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger] constructor with an arbitrary
     * predicate, or via the named factories in [ ]'s subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [Flight][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        drivetrain.setDefaultRunCommand{
            //swerveDrive(controller.swerveOutput)
            swerveDrive(0.5,0.2,0.3)
            //topLeft.io.setTurnVoltage(2.0.volts)
            //topRight.io.setTurnVoltage(2.0.volts)
            //bottomLeft.io.setTurnVoltage(2.0.volts)
            //bottomRight.io.setTurnVoltage(2.0.volts)
        }
    }

    val autonomousCommand: Command
        get() = DoNothing()
}
