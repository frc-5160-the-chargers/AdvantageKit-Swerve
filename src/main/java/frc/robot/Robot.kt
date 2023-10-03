// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot


import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.chargers.advantagekitextensions.NTSafePublisher
import frc.chargers.advantagekitextensions.logChargerLibMetadata
import frc.chargers.advantagekitextensions.logGitDirty
import frc.chargers.advantagekitextensions.startCommandLog
import frc.robot.BuildConstants.*
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import java.nio.file.Files
import java.nio.file.Path

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
object Robot : LoggedRobot() {
    private var m_autonomousCommand: Command? = null

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {

        val logger = Logger.getInstance()
        setUseTiming(
            RobotBase.isReal() || !IS_REPLAY
        )

        logger.apply{
            recordMetadata(
                "Robot", if (RobotBase.isReal()) "REAL" else if (IS_REPLAY) "REPLAY" else "SIM"
            )
            recordMetadata("ProjectName", MAVEN_NAME)
            recordMetadata("BuildDate", BUILD_DATE)
            recordMetadata("GitSHA", GIT_SHA)
            recordMetadata("GitBranch", GIT_BRANCH)
            // custom extension function which replaces a when branch for logging git dirty
            logGitDirty(DIRTY)
            // custom extension function that logs ChargerLib's BUILD_DATE, GIT_SHA, GIT_BRANCH and GIT_DIRTY
            logChargerLibMetadata()

            // real robot
            if (RobotBase.isReal()){
                if (Files.exists(Path.of(LOG_FOLDER))){
                    addDataReceiver(WPILOGWriter(LOG_FOLDER))
                }
                addDataReceiver(NTSafePublisher())
            }else if (IS_REPLAY){
                // replay mode; sim
                val path = LogFileUtil.findReplayLog()
                setReplaySource(WPILOGReader(path))
                addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replayed")))
            }else{
                // sim mode
                logger.addDataReceiver(NTSafePublisher())
                // maybe add DriverStationSim? idk
            }

            // no more configuration from this point on
            start()
        }

        LiveWindow.disableAllTelemetry()

        // inits robotContainer; objects are only initialized on first access
        RobotContainer

        // custom extension function in chargerlib
        CommandScheduler.getInstance().startCommandLog()
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
        m_autonomousCommand = RobotContainer.autonomousCommand
        m_autonomousCommand?.schedule()
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}
    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        m_autonomousCommand?.cancel()
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {}
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {}

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {}
}
