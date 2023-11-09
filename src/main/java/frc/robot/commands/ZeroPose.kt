package frc.robot.commands

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.buildCommand
import frc.chargers.hardware.subsystems.drivetrain.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.geometry.UnitPose2d
import frc.robot.RESET_POSE_ON_STARTUP

fun EncoderHolonomicDrivetrain.zeroPose(): Command = buildCommand(
    name = "DrivetrainZeroPose"
) {
    runOnce(this@zeroPose){
        if (RobotBase.isSimulation() && RESET_POSE_ON_STARTUP){
            poseEstimator.resetPose(UnitPose2d())
        }
    }
}