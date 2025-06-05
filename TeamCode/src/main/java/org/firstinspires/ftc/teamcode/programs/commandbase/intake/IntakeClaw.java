package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.programs.commandbase.claw.UpdateClawState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;

public class IntakeClaw extends SequentialCommandGroup {
    public IntakeClaw(){
        super(
             //   new AlignMecanum(),
             //   new WaitCommand(500),
                new Servo1SetAngle(),
                new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.EXTEND, 4),
              //  new Servo4Extend(),`
              //  new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.PICKUP0, 0),
             //   new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.PICKUP3, 3)
        );
    }
}
