package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import static org.firstinspires.ftc.teamcode.programs.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.programs.utils.Constants.ROBOT_SPEED;
import java.util.Arrays;
import java.util.List;

/**
 * Robot centric drive is a drive system that allows the robot to move in a direction relative to the robot
 *
 * @version 1.0
 */
@Config
public class Mecanum {
    double reverse = 1.0;
    public static double powerReduction = 5;
    public double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower, normalizer, x, y, r;

    private final Robot robot = Robot.getInstance();
    public void motorMath(GamepadEx gamepad){
        x = -gamepad.getLeftX();
        y = -gamepad.getLeftY();
        r = -gamepad.getRightX();

        x = addons(x)*reverse;
        y = addons(y)*reverse;
        r = addons(r);

        normalizer = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);

        leftFrontPower = (y + x + r) / normalizer;
        rightFrontPower = (y - x - r) / normalizer;
        leftRearPower = (y - x + r) / normalizer;
        rightRearPower = (y + x - r) / normalizer;
    }


    public void teleop(GamepadEx gamepad) {
        motorMath(gamepad);

        robot.leftFront.setPower(leftFrontPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightRear.setPower(rightRearPower);
    }

    public void slowMotion(GamepadEx gamepad)
    {
        motorMath(gamepad);

        robot.leftFront.setPower(leftFrontPower/powerReduction);
        robot.rightFront.setPower(rightFrontPower/powerReduction);
        robot.leftRear.setPower(leftRearPower/powerReduction);
        robot.rightRear.setPower(rightRearPower/powerReduction);
    }

    public void spin() {
        robot.leftFront.setPower(0.5);
        robot.leftRear.setPower(-0.5);
        robot.rightFront.setPower(-0.5);
        robot.rightRear.setPower(0.5);
    }

    public void loop(GamepadEx gamepad){
        if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            slowMotion(gamepad);
        }
        else{
            teleop(gamepad);
        }
    }

    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * ROBOT_SPEED;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("---ROBOT CENTRIC DRIVE---");

        telemetry.addData("Direction Multiplier: ", reverse);
        telemetry.addData("Speed Multiplier: ", ROBOT_SPEED);

        telemetry.addData("LeftRear Position: ", robot.leftRear.getCurrentPosition());
        telemetry.addData("RightRear Position: ", robot.rightRear.getCurrentPosition());
        telemetry.addData("LeftFront Position: ", robot.leftFront.getCurrentPosition());
        telemetry.addData("RightFront Position: ", robot.rightFront.getCurrentPosition());

        telemetry.addData("LeftRear Power: ", robot.leftRear.getPower());
        telemetry.addData("RightRear Power: ", robot.rightRear.getPower());
        telemetry.addData("LeftFront Power: ", robot.leftFront.getPower());
        telemetry.addData("RightFront Power: ", robot.rightFront.getPower());
    }

}