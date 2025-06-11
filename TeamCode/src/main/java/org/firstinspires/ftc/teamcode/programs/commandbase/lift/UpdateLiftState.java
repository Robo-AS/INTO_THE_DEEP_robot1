package org.firstinspires.ftc.teamcode.programs.commandbase.lift;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class UpdateLiftState extends InstantCommand {
    public UpdateLiftState(Lift.LiftState state){
        super(
                () -> Robot.getInstance().getInstanceLift().update(state)
        );
    }
}