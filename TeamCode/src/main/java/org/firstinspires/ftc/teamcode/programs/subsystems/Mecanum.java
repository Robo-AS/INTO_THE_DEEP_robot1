package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import static org.firstinspires.ftc.teamcode.programs.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.programs.utils.Constants.ROBOT_SPEED;

import java.util.function.DoubleSupplier;

/**
 * Robot centric drive is a drive system that allows the robot to move in a direction relative to the robot
 *
 * @version 1.0
 */
@Config
public class Mecanum extends SubsystemBase {
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
        robot.mecanum.motorMath(gamepad);

        robot.leftFront.setPower(leftFrontPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightRear.setPower(rightRearPower);
    }

    public void slowMotion(GamepadEx gamepad)
    {
        robot.mecanum.motorMath(gamepad);

        robot.leftFront.setPower(leftFrontPower/powerReduction);
        robot.rightFront.setPower(rightFrontPower/powerReduction);
        robot.leftRear.setPower(leftRearPower/powerReduction);
        robot.rightRear.setPower(rightRearPower/powerReduction);
    }

    public void loop(GamepadEx gamepad){
        if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            robot.mecanum.slowMotion(gamepad);
        }
        else{
            robot.mecanum.teleop(gamepad);
        }
    }

    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * ROBOT_SPEED;
    }
    public void stop() {
        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);
    }
    public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        Robot.getInstance().getInstanceFollower().setTeleOpMovementVectors(xSupplier.getAsDouble(), ySupplier.getAsDouble(), 0, false);
    }
}