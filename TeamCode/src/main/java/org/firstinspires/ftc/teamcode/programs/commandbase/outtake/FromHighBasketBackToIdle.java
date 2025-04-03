package org.firstinspires.ftc.teamcode.programs.commandbase.outtake;

import org.firstinspires.ftc.teamcode.programs.commandbase.lift.UpdateLiftState;
import org.firstinspires.ftc.teamcode.programs.commandbase.arm.UpdateArmState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class FromHighBasketBackToIdle extends SequentialCommandGroup {
    public FromHighBasketBackToIdle(){
        super(
                new UpdateLiftState(Lift.LiftState.SCORE),
                new UpdateArmState(Arm.ArmState.REAR),
                new WaitCommand(500),
                new UpdateArmState(Arm.ArmState.FRONT),
                new UpdateLiftState(Lift.LiftState.IDLE)
        );
    }
}
