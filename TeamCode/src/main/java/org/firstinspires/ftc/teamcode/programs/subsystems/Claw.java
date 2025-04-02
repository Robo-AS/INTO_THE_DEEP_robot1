package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo testServo;
    public Claw(HardwareMap hardwareMap){
        testServo = hardwareMap.get(Servo.class, "testServo");
    }

    public void loop(GamepadEx gamepad){
        if(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            testServo.setPosition(0.55);
        }
    }

    public void adjustForSample(double sampleAngle) {
        double servoAngle = 0.5 - (sampleAngle / 180.0);
        servoAngle = Math.max(0.0, Math.min(1.0, servoAngle));

        testServo.setPosition(servoAngle);
    }
}

