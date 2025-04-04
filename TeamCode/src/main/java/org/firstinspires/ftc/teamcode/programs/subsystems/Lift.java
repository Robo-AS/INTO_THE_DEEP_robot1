package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;


@Config
public class Lift extends SubsystemBase {
    private static Lift instance = null;
    public CachingDcMotorEx leftSlider, rightSlider;

    public enum LiftState{
        IDLE,
        SCORE
    }

    public LiftState liftState = LiftState.IDLE;
    public static int IDLE = 0;
    public static int SCORE = 0;




    private PIDController sliders_pid;
    public static int targetPosition = 0; //variable is static because it needs to be changed in dashboard
    public static int currentPosition = 0;//variable is static because it needs to be changed in dashboard
    public static double p_sliders = 0, d_sliders = 0, i_sliders = 0;

    public Lift(){
        sliders_pid = new PIDController(p_sliders, d_sliders, i_sliders);
    }

    public static Lift getInstance() {
        if (instance == null) {
            instance = new Lift();
        }
        return instance;
    }

    public void initializeHardware(final HardwareMap hardwareMap){

        leftSlider = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftSlider"));
        rightSlider = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightSlider"));

        leftSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlider.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void initialize(){
        sliders_pid.reset();
        liftState = LiftState.IDLE;
        targetPosition = 0;
    }

    public void loop(){
        currentPosition = rightSlider.getCurrentPosition();

        sliders_pid.setPID(p_sliders, d_sliders, i_sliders);
        double power = sliders_pid.calculate(currentPosition, targetPosition);
        rightSlider.setPower(power);
        leftSlider.setPower(power);
    }

    public void update(LiftState state){
        liftState = state;
        switch (liftState){
            case IDLE:
                targetPosition = IDLE;
                break;

            case SCORE:
                targetPosition = SCORE;
                break;
        }
    }

    public double getTargetPosition(){
        return targetPosition;
    }

}
