package frc.robot.commands

/*
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.chargers.commands.buildCommand
import frc.chargers.commands.repeatFor
import frc.chargers.wpilibextensions.fpgaTimestamp


/*
Wait for 0.05 seconds for trigger to return false.
If trigger is still true during that time, END the command.


Then, while the trigger

 */




fun activateOnMultipleClicks(numExtraClicks: Int, command: Command, trigger: Trigger): Command =
    buildCommand{
        var numClicksExecuted = 0
        +(buildCommand{
            waitFor(0.01.seconds)




        }.repeatFor(numExtraClicks))

    }


class ExtraClickCommand(val command: Command, val trigger: Trigger): CommandBase() {
    var extraClickActivated = false
    var extraClickTimeout: Time = 0.05.seconds

    var extraClickStartTime: Time = Time(0.0)


    var commandFinished: Boolean = false

    override fun initialize(){
        extraClickStartTime = fpgaTimestamp()
    }

    override fun execute(){
        if (fpgaTimestamp() - extraClickStartTime > extraClickTimeout){
            commandFinished = true
        }
    }



    override fun isFinished(): Boolean = commandFinished

} */
