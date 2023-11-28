package frc.robot.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.inputdevices.ChargerController
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.chargers.wpilibextensions.ratelimit.ScalarRateLimiter
import org.littletonrobotics.junction.Logger
import kotlin.math.abs


object DriverController: ChargerController(port = 0, deadband = 0.1){

    /* Top-Level constants */
    private const val AIM_TO_TARGET_ENABLED = true
    private val aimToTargetPIDConstants = PIDConstants(1.2,0.0,0.0)

    // coefficients go from greatest power to smallest.
    private val driveMultiplierFunction = Polynomial(0.2,0.0,0.5,0.0).preserveSign()
    private val rotationMultiplierFunction = Polynomial(-0.3,0.0,-0.3,0.0).preserveSign()

    private val forwardLimiter: ScalarRateLimiter? = null /*ScalarRateLimiter(
        Scalar(0.6) / 1.seconds,
        Scalar(-0.8) / 1.seconds,
        Scalar(0.0) / 1.seconds
    )*/
    private val strafeLimiter: ScalarRateLimiter? = null /*ScalarRateLimiter(
        Scalar(0.6) / 1.seconds,
        Scalar(-0.8) / 1.seconds,
        Scalar(0.0) / 1.seconds
    )*/
    private val rotationLimiter: ScalarRateLimiter? = null

    private val turboModeMultiplier = 1.0..2.0
    private val precisionModeDivider = 1.0..4.0





    /* Private implementation variables  */
    private var currentHeading: Angle = 0.0.degrees
    private var aimToAngleController = UnitSuperPIDController(
        aimToTargetPIDConstants,
        getInput = {currentHeading.inputModulus(0.0.degrees..360.degrees)},
        outputRange = Scalar(-0.7)..Scalar(0.7),
        target = 0.0.degrees,
        continuousInputRange = 0.0.degrees..360.degrees
    )




    /* Public API */
    var targetHeading: Angle? = null
    val pointNorthButton: Trigger = y()
    val pointSouthButton: Trigger = a()
    val pointEastButton: Trigger = x()
    val pointWestButton: Trigger = b()
    val headingZeroButton: Trigger = back()

    fun swerveOutput(robotHeading: Angle? = null): ChassisPowers{
        var forward = driveMultiplierFunction( leftY.withScaledDeadband() )
        var strafe = driveMultiplierFunction( leftX.withScaledDeadband() )
        var rotation = rotationMultiplierFunction( rightX.withScaledDeadband() )

        if (forwardLimiter != null) forward = forwardLimiter.calculate(forward)

        if (strafeLimiter != null) strafe = strafeLimiter.calculate(strafe)

        if (rotationLimiter != null) rotation = rotationLimiter.calculate(rotation)


        var turbo = abs(leftTriggerAxis).mapTriggerValue(turboModeMultiplier)
        var precision = 1 / abs(rightTriggerAxis).mapTriggerValue(precisionModeDivider)

        if (turbo < 1.0 || precision.isInfinite() || precision.isNaN()){
            turbo = 1.0
        }
        if (precision.isInfinite() || precision.isNaN() || precision > 1.0 || precision == 0.0){
            precision = 1.0
        }

        
        if (AIM_TO_TARGET_ENABLED){
            if (robotHeading != null){
                currentHeading = robotHeading
            }

            targetHeading?.let{
                aimToAngleController.target = it
                rotation = aimToAngleController.calculateOutput().siValue
            }
        }

        Logger.getInstance().apply{
            recordOutput("Drivetrain(Swerve)/rotation output", rotation)
            recordOutput("Drivetrain(Swerve)/xPower", forward)
            recordOutput("Drivetrain(Swerve)/yPower", strafe)
        }

        return ChassisPowers(
            xPower = forward * turbo * precision,
            yPower = strafe * turbo * precision,
            rotationPower = rotation * precision
        )
    }



}