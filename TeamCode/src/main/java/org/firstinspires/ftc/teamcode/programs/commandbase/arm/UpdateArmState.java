package org.firstinspires.ftc.teamcode.programs.commandbase.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class UpdateArmState extends InstantCommand {
    public UpdateArmState(Arm.ArmState state){
        super(
                () -> Robot.getInstance().getInstanceArm().update(state)
        );
    }
}