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

    private PIDController liftPID;

    public static double ilift = 0.11, plift = 0.007, dlift = 0.00006;
    private List<DcMotorEx> sliders;

    public int targetPosition = 0, toggleLift = 0;
    public static int currentPosition;

    public enum LiftState{
        SCORE,
        LOW_BASKET,
        HIGH_BASKET,
        LOW_RUNG,
        PUT_SPECIMEN,
        UP_FOR_IDLE,
        HIGH_BASKET_AUTO,
        IDLE
    }

    public LiftState liftState = LiftState.IDLE;

    public static int SCORE = 100;
    public static int LOW_BASKET = 400;
    public static int LOW_RUNG = 0;
    public static int PUT_SPECIMEN = 160;
    public static int UP_FOR_IDLE = 300;
    public static int HIGH_BASKET_AUTO = 840;
    public static int IDLE = 0;

    public Lift(HardwareMap hardwareMap){
        liftPID = new PIDController(plift, ilift, dlift);
    }

    public void initializeLift(HardwareMap hardwareMap){
        leftSlider = hardwareMap.get(DcMotorEx.class, "leftSlider");
        rightSlider = hardwareMap.get(DcMotorEx.class, "rightSlider");

        sliders = Arrays.asList(leftSlider, rightSlider);

        for(DcMotorEx slider : sliders){
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void initialize() {
        liftPID.reset();
        liftState = LiftState.IDLE;
        targetPosition = 0;
    }

    public void loop(GamepadEx gamepadd)
    {
        currentPosition = leftSlider.getCurrentPosition();

        double power = liftPID.calculate(currentPosition, targetPosition);
        liftPID.setPID(plift, ilift, dlift);
        leftSlider.setPower(power);
        rightSlider.setPower(power);

        if(liftState == LiftState.IDLE)
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
        }

    }
    public void updateLiftState(GamepadEx gamepad){
        if(gamepad.getButton(GamepadKeys.Button.Y) && toggleLift == 0) {
            liftState = LiftState.SCORE;
            toggleLift = 1;
        }
        else if(gamepad.getButton(GamepadKeys.Button.Y) && toggleLift == 1){
            liftState = LiftState.IDLE;
            toggleLift = 0;
        }
    }


}
