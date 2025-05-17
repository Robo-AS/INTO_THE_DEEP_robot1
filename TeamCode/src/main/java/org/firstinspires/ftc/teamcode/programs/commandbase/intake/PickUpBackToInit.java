package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.arm.UpdateArmState;
import org.firstinspires.ftc.teamcode.programs.commandbase.claw.UpdateClawState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;

public class PickUpBackToInit extends SequentialCommandGroup {
    public PickUpBackToInit(){
        super(
                new UpdateClawState(Claw.ClawState.OPEN, 2),
                new Servo1SetAngle(),
                new UpdateArmState(Arm.ArmState.PICKUP),
                new WaitCommand(1000),
                new UpdateClawState(Claw.ClawState.CLOSE, 2),
                new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.SCORE3, 3),
                new UpdateArmState(Arm.ArmState.FRONT),
                new UpdateClawState(Claw.ClawState.RETRACT, 4),
                new UpdateClawState(Claw.ClawState.IDLE0, 0),
                new UpdateClawState(Claw.ClawState.OPEN, 1)
        );
    }
}
