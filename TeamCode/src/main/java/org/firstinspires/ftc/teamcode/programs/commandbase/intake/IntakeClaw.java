package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.arm.UpdateArmState;
import org.firstinspires.ftc.teamcode.programs.commandbase.claw.UpdateClawState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;

public class IntakeClaw extends SequentialCommandGroup {
    public IntakeClaw(){
        super(
                new UpdateClawState(Claw.ClawState.EXTEND, 4),
                new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.PICKUP0, 0),
                new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.PICKUP3, 3),
                new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.OPEN, 2),
                new WaitCommand(500),
                new Servo1SetAngle(),
                new UpdateArmState(Arm.ArmState.PICKUP),
                new WaitCommand(1500),
                new UpdateClawState(Claw.ClawState.CLOSE, 2),
                new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.SCORE3, 3),
                new WaitCommand(500),
                new UpdateArmState(Arm.ArmState.FRONT),
                new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.RETRACT, 4)
        );
    }
}
