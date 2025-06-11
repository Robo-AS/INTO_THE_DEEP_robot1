package org.firstinspires.ftc.teamcode.programs.commandbase.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Servo1SetAngle extends InstantCommand {
    public Servo1SetAngle(){
        super(
                () -> Robot.getInstance().servo1.setPosition(Robot.getInstance().camera.sampleAngle())
        );
    }
}
