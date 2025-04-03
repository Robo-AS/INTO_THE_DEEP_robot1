package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Arm extends SubsystemBase{
    private Robot robot = Robot.getInstance();

    public enum ArmState{
        FRONT, REAR
    }

    public ArmState armState = ArmState.FRONT;
    public static int FRONT = 0;
    public static int REAR = 10;


    private PIDController arm_pid;
    public static int targetPosition = 0, currentPosition = 0;
    public static double p_arm = 0, i_arm = 0, d_arm = 0;

    public Arm(){
        arm_pid = new PIDController(p_arm, i_arm, d_arm);
    }

    public void initialize()
    {
        arm_pid.reset();
        armState = ArmState.FRONT;
        targetPosition = 0;
    }

    public void loop(){
        currentPosition = robot.armMotor.getCurrentPosition();

        arm_pid.setPID(p_arm, i_arm ,d_arm);

        double power = arm_pid.calculate(currentPosition, targetPosition);
        robot.leftSlider.setPower(power);
        robot.rightSlider.setPower(power);
    }

    public void update(ArmState state){
        armState = state;
        switch(armState){
            case FRONT:
                targetPosition = FRONT;
                break;

            case REAR:
                targetPosition = REAR;
                break;
        }
    }
}
