package frc.robot



import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.GitData
import frc.robot.BuildConstants.*

val ROBOT = ChargerRobot(
    { RobotContainer },
    GitData(
        projectName = MAVEN_NAME,
        buildDate = BUILD_DATE,
        sha = GIT_SHA,
        branch = GIT_BRANCH,
        dirty = DIRTY
    ),
    config = CONFIG
)
