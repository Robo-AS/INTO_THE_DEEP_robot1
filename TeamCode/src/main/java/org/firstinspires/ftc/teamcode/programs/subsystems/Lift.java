package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;


@Config
public class Lift extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum LiftState{
        IDLE,
        SCORE
    }

    public LiftState liftState = LiftState.IDLE;
    public static int IDLE = 0;
    public static int SCORE = 0;




    private PIDController sliders_pid;
    public static int targetPosition = 0;
    public static int currentPosition = 0;
    public static double p_sliders = 0, d_sliders = 0, i_sliders = 0;

    public Lift(){
        sliders_pid = new PIDController(p_sliders, d_sliders, i_sliders);
    }

    public void initialize(){
        sliders_pid.reset();
        liftState = LiftState.IDLE;
        targetPosition = 0;
    }

    public void loop(){
        currentPosition = robot.rightSlider.getCurrentPosition();

        sliders_pid.setPID(p_sliders, d_sliders, i_sliders);
        double power = sliders_pid.calculate(currentPosition, targetPosition);
        robot.rightSlider.setPower(power);
        robot.leftSlider.setPower(power);
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
