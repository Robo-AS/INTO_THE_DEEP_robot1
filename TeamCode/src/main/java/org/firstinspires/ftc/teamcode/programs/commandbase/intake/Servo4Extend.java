package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Servo4Extend extends InstantCommand {
    public Servo4Extend(){
        super(
                () -> Robot.getInstance().servo4.setPosition(Robot.getInstance().camera.sliderPositionFromDistance())
        );
    }
}
