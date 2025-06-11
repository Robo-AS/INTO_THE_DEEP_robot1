package org.firstinspires.ftc.teamcode.programs.commandbase.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.arm.UpdateArmState;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;

public class ArmDownTest extends SequentialCommandGroup {
    public ArmDownTest(){
        super(
            new UpdateArmState(Arm.ArmState.PICKUP)
        );
    }
}
