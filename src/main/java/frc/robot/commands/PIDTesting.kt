package frc.robot.commands

import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.utils.math.units.rem

class StockPIDcontrollerSwerveTest(
    drivetrain: EncoderHolonomicDrivetrain
): CommandBase() {
    val pidController = PIDController(4.0,0.0,0.0).also{
        it.enableContinuousInput(0.0.degrees.siValue, 360.degrees.siValue)
    }

    val module = drivetrain.topLeft


    override fun execute(){
        println(module.currentDirection.inUnit(degrees))
        module.io.setTurnVoltage(pidController.calculate(module.currentDirection.siValue, 0.0).ofUnit(volts))
    }
}

class ChargerPIDcontrollerSwerveTest(
    drivetrain: EncoderHolonomicDrivetrain
): CommandBase(){

    val module = drivetrain.topLeft
    val pidController = UnitSuperPIDController(
        PIDConstants(4.0,0.0,0.0),
        {if (module.currentDirection < 0.degrees) module.currentDirection % 360.degrees + 360.degrees else module.currentDirection % 360.degrees },
        target = 0.0.degrees,
        outputRange = -12.volts..12.volts,
    )

    override fun execute(){
        println(module.currentDirection.inUnit(degrees))
        module.io.setTurnVoltage(pidController.calculateOutput())
    }
}