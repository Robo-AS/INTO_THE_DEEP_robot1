package org.firstinspires.ftc.teamcode.programs.commandbase.claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class UpdateClawState extends InstantCommand {
    public UpdateClawState(Claw.ClawState state, int x){
        super(
                () -> Robot.getInstance().getInstanceClaw().update(state, x)
        );
    }
}