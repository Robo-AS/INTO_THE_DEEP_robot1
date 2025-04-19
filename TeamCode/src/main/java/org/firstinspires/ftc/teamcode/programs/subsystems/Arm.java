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
        FRONT, REAR, PICKUP
    }

    public ArmState armState = ArmState.FRONT;
    public static int FRONT = 0;
    public static int PICKUP = -100;
    public static int REAR = 1300;


    private PIDController arm_pid;
    public static int targetPosition = 0, currentPosition = 0;
    public static double p_arm = 0.01, i_arm = 0.009, d_arm = 0.0001;

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
        robot.armMotor.setPower(power);
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
            case PICKUP:
                targetPosition = PICKUP;
                break;
        }
    }

    public double getTargetPosition(){
        return targetPosition;
    }
}
