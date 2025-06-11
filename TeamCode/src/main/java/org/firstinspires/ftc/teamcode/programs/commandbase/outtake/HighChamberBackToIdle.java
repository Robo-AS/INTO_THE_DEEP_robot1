package org.firstinspires.ftc.teamcode.programs.commandbase.outtake;

import org.firstinspires.ftc.teamcode.programs.commandbase.claw.UpdateClawState;
import org.firstinspires.ftc.teamcode.programs.commandbase.lift.UpdateLiftState;
import org.firstinspires.ftc.teamcode.programs.commandbase.arm.UpdateArmState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class HighChamberBackToIdle extends SequentialCommandGroup {
    public HighChamberBackToIdle(){
        super(
                new UpdateArmState(Arm.ArmState.FRONT),
                new WaitCommand(1000),
                new UpdateClawState(Claw.ClawState.OPEN, 1),
                new WaitCommand(1000),
                new UpdateClawState(Claw.ClawState.UPRIGHT3, 3),
                new WaitCommand(500),
                new UpdateLiftState(Lift.LiftState.SPECUP),
                new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.SPEC, 0),
                new WaitCommand(1000),
                new UpdateLiftState(Lift.LiftState.SPECDOWN),
                new WaitCommand(500),
                new UpdateClawState(Claw.ClawState.OPEN, 2),
                new WaitCommand(500),
                new UpdateLiftState(Lift.LiftState.IDLE)
        );
    }
}
