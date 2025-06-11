package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.programs.commandbase.claw.UpdateClawState;
import org.firstinspires.ftc.teamcode.programs.commandbase.mecanum.AlignMecanum;
import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class IntakeClaw extends SequentialCommandGroup {
    public IntakeClaw(){
        super(
                new AlignMecanum(Robot.getInstance().camera, Robot.getInstance().mecanum),
                new WaitCommand(500),
                new Servo1SetAngle(),
                new UpdateClawState(Claw.ClawState.EXTEND, 4),
                new UpdateClawState(Claw.ClawState.PICKUP0, 0),
                new UpdateClawState(Claw.ClawState.PICKUP3, 3)
        );
    }
}
