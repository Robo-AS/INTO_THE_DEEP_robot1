package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

public class Claw extends SubsystemBase {

    Robot robot = Robot.getInstance();
    public enum ClawState{
        IDLE0,
        PICKUP0,
        OPEN,
        CLOSE,
        PICKUP3,
        SCORE3,
        EXTEND,
        RETRACT
    }

    public static double IDLE0 = 0;
    public static double OPEN = 0.5;
    public static double CLOSE = 0.25;
    public static double PICKUP0 = 0.45;
    public static double PICKUP3 = 0.25;
    public static double SCORE3 = 0.75;
    public static double EXTEND = 0;
    public static double RETRACT = 0.5;

    double[] targetPosition = new double[5];

    ClawState[] clawState = new ClawState[5];

    public void initialize(){
        robot.servo0.setPosition(1);
        robot.servo1.setPosition(0.5);
        robot.servo2.setPosition(0.25);
        robot.servo3.setPosition(0.15);
        robot.servo4.setPosition(0.5);
    }

    public void loop(){
        robot.servo0.setPosition(targetPosition[0]);
        robot.servo2.setPosition(targetPosition[2]);
        robot.servo3.setPosition(targetPosition[3]);
        robot.servo4.setPosition(targetPosition[4]);
    }

    public void update(Claw.ClawState state, int x){
        clawState[x] = state;
        switch (clawState[x]){
            case PICKUP0:
                targetPosition[x] = PICKUP0;
                break;
            case IDLE0:
                targetPosition[x] = IDLE0;
                break;
            case OPEN:
                targetPosition[x] = OPEN;
                break;
            case CLOSE:
                targetPosition[x] = CLOSE;
                break;
            case PICKUP3:
                targetPosition[x] = PICKUP3;
                break;
            case SCORE3:
                targetPosition[x] = SCORE3;
                break;
            case EXTEND:
                targetPosition[x] = EXTEND;
                break;
            case RETRACT:
                targetPosition[x] = RETRACT;
                break;
        }
    }

    public void pointClawToSample(int sampleX, int frameWidth, double fovDegrees) {
        int imageCenterX = frameWidth / 2;
        double pixelOffset = sampleX - imageCenterX;
        double degreesPerPixel = fovDegrees / frameWidth;

        double angleOffset = pixelOffset * degreesPerPixel;

        double baseAngle = 90;
        double desiredAngle = baseAngle + angleOffset;
        desiredAngle = Math.max(0, Math.min(180, desiredAngle));

        double servoPosition = desiredAngle / 180.0;
        robot.servo1.setPosition(servoPosition);
    }
}
