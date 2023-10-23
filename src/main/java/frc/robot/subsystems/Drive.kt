package frc.robot.subsystems


import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.constants.MK4i
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.realEncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.simEncoderHolonomicDrivetrain
import frc.chargers.hardware.swerve.sparkMaxSwerveMotors
import frc.chargers.hardware.swerve.swerveCANcoders
import frc.robot.DriveConstants

val Drivetrain: EncoderHolonomicDrivetrain by lazy{
    if (RobotBase.isReal()){
        realEncoderHolonomicDrivetrain(
            turnMotors = sparkMaxSwerveMotors(
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
            driveMotors = sparkMaxSwerveMotors(
                topLeft = neoSparkMax(0),
                topRight = neoSparkMax(1),
                bottomLeft = neoSparkMax(0),
                bottomRight = neoSparkMax(0)
            ),
            controlScheme = DriveConstants.realControlScheme,
            driveGearRatio = MK4i.GEAR_RATIO_L2,
            turnGearRatio = MK4i.TURN_GEAR_RATIO,
            trackWidth = DriveConstants.trackWidth,
            wheelBase = DriveConstants.wheelBase,
            wheelDiameter = MK4i.WHEEL_DIAMETER
        )
    }else{
        simEncoderHolonomicDrivetrain(
            turnGearbox = DCMotor.getNEO(1),
            driveGearbox = DCMotor.getNEO(1),
            turnGearRatio = MK4i.TURN_GEAR_RATIO,
            driveGearRatio = MK4i.GEAR_RATIO_L2,
            controlScheme = DriveConstants.simControlScheme,
            trackWidth = DriveConstants.trackWidth,
            wheelBase = DriveConstants.wheelBase,
            wheelDiameter = MK4i.WHEEL_DIAMETER
        )
    }
}

