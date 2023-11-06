package frc.robot.subsystems


import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.realEncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.simEncoderHolonomicDrivetrain
import frc.robot.*

val Drivetrain: EncoderHolonomicDrivetrain by lazy{
    if (RobotBase.isReal()){
        realEncoderHolonomicDrivetrain(
            turnMotors = DriveHardware.turnMotors,
            turnEncoders = DriveHardware.encoders,
            driveMotors = DriveHardware.driveMotors,
            controlScheme = REAL_CONTROL_SCHEME,
            constants = DRIVE_CONSTANTS,
            gyro = Gyro
        )
    }else{
        simEncoderHolonomicDrivetrain(
            turnGearbox = DCMotor.getNEO(1),
            driveGearbox = DCMotor.getNEO(1),
            controlScheme = SIM_CONTROL_SCHEME,
            constants = DRIVE_CONSTANTS
        )
    }
}

