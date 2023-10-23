package frc.robot.subsystems

import frc.chargers.hardware.subsystems.posemonitors.SwervePoseMonitor

val PoseEstimator by lazy{
    SwervePoseMonitor(Drivetrain)
}