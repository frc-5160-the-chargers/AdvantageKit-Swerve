package frc.robot.commands

import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain

class SwerveTurnMotorTest(
    val drivetrain: EncoderHolonomicDrivetrain
): CommandBase() {

    init{
        addRequirements(drivetrain)
    }

    override fun execute() {
        drivetrain.apply{
            topLeft.io.setTurnVoltage(0.5.volts)
            topRight.io.setTurnVoltage(-0.5.volts)
            bottomLeft.io.setTurnVoltage(-0.5.volts)
            bottomRight.io.setTurnVoltage(0.5.volts)
        }
    }
    override fun isFinished(): Boolean = false
}