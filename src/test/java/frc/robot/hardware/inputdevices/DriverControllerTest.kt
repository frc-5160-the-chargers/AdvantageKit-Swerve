package frc.robot.hardware.inputdevices

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.RobotBase
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class DriverControllerTest{
    @BeforeEach
    internal fun setup(){
        assert(HAL.initialize(500,0))
    }


    @Test
    fun `check if real`(){
        assertEquals(RobotBase.isSimulation(),true)
    }
}