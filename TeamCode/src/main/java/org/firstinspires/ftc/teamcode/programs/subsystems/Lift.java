package org.firstinspires.ftc.teamcode.programs.subsystems;

import static org.firstinspires.ftc.teamcode.programs.utils.Constants.ROBOT_SPEED;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@Config
public class Lift{
    private DcMotorEx leftSlider, rightSlider;

    private PIDController liftPIDLeft, liftPIDRight;

    public static double i_lift_left = 0.11, p_lift_left = 0.04, d_lift_left = 0.00006, i_lift_right = 0.17, p_lift_right = 0.01, d_lift_right =  0.00012;
    private List<DcMotorEx> sliders;

    public int targetPosition = 0, toggleLift = 0;
    public int currentPositionLeft, currentPositionRight;

    public enum LiftState{
        SCORE,
        LOW_BASKET,
        HIGH_BASKET,
        LOW_RUNG,
        PUT_SPECIMEN,
        UP_FOR_IDLE,
        HIGH_BASKET_AUTO,
        IDLE,
        DOWN
    }

    public LiftState liftState = LiftState.IDLE;

    public static int SCORE = 100;
    public static int LOW_BASKET = 400;
    public static int LOW_RUNG = 0;
    public static int PUT_SPECIMEN = 160;
    public static int UP_FOR_IDLE = 300;
    public static int HIGH_BASKET_AUTO = 840;
    public static int IDLE = 0;
    public static int DOWN = -5;

    public Lift(HardwareMap hardwareMap){
        liftPIDLeft = new PIDController(p_lift_left, i_lift_left, d_lift_left);
        liftPIDRight = new PIDController(p_lift_right, i_lift_right, d_lift_right);
    }

    public void initializeLift(HardwareMap hardwareMap){
        leftSlider = hardwareMap.get(DcMotorEx.class, "leftSlider");
        rightSlider = hardwareMap.get(DcMotorEx.class, "rightSlider");

        leftSlider.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        sliders = Arrays.asList(leftSlider, rightSlider);

        for(DcMotorEx slider : sliders){
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void initialize() {
        liftPIDLeft.reset();
        liftPIDRight.reset();
        liftState = LiftState.DOWN;
        targetPosition = 0;
    }

    public void loop(GamepadEx gamepadd)
    {
        currentPositionLeft = leftSlider.getCurrentPosition();
        currentPositionRight= rightSlider.getCurrentPosition();

        double powerLeft = liftPIDLeft.calculate(currentPositionLeft, targetPosition);
        double powerRight = liftPIDRight.calculate(currentPositionRight, targetPosition);

        liftPIDLeft.setPID(p_lift_left, i_lift_left, d_lift_left);
        liftPIDRight.setPID(p_lift_right, i_lift_right, d_lift_right);

        leftSlider.setPower(powerLeft);
        rightSlider.setPower(powerRight);

        if(liftState == LiftState.DOWN || liftState == LiftState.IDLE)
            updateLiftState(gamepadd);

        update(liftState);
    }

    public void update(LiftState state){
        liftState = state;
        switch (liftState){
            case LOW_BASKET:
                targetPosition = LOW_BASKET;
                break;

            case SCORE:
                targetPosition = SCORE;
                break;

            case LOW_RUNG:
                targetPosition = LOW_RUNG;
                break;

            case PUT_SPECIMEN:
                targetPosition = PUT_SPECIMEN;
                break;
            case UP_FOR_IDLE:
                targetPosition = UP_FOR_IDLE;
                break;

            case HIGH_BASKET_AUTO:
                targetPosition = HIGH_BASKET_AUTO;
                break;
            case IDLE:
                targetPosition = IDLE;
                break;
            case DOWN:
                targetPosition = DOWN;
                break;
        }

    }
    public void updateLiftState(GamepadEx gamepad){
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y)){
                liftState = LiftState.SCORE;
        }
    }

    public void telemetryLift(Telemetry telemetry){
        telemetry.addData("leftSlider: ", leftSlider.getCurrentPosition());
        telemetry.update();
        telemetry.addData("rightSlider: ", rightSlider.getCurrentPosition());
        telemetry.update();
    }
}
