package org.firstinspires.ftc.teamcode.programs.commandbase.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.claw.UpdateClawState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;

public class TestServo extends SequentialCommandGroup {
    public TestServo(){
        super(
                // new UpdateLiftState(Lift.LiftState.PICKUP),
                new UpdateClawState(Claw.ClawState.PICKUP0, 3),
                new UpdateClawState(Claw.ClawState.IDLE0, 3)
        );
    }
}
