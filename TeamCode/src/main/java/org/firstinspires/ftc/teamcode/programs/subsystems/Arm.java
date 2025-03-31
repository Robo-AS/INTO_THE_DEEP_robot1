package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

import java.util.Arrays;
import java.util.List;

public class Arm {
    private PIDController armPID;

    private DcMotorEx armMotor;

    public static double parm = 0.2, iarm = 0.5, darm = 0.0004;

    public int targetPosition, toggleArm = 0;

    public static int currentPosition;

    enum ArmState{
        FRONT,
        REAR
    }

    public ArmState armstate = ArmState.FRONT;

    public static int FRONT = 0;
    public static int REAR = 1;

    public Arm(HardwareMap hardwareMap){
        armPID = new PIDController(parm, iarm, darm);
    }

    public void initialize()
    {
        armPID.reset();
        armstate = ArmState.FRONT;
        targetPosition = 0;
    }

    public void initializeHardware(HardwareMap hardwareMap)
    {
        armMotor = hardwareMap.get(DcMotorEx.class, "Circular");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(GamepadEx gamepad){
        currentPosition = armMotor.getCurrentPosition();

        double power = armPID.calculate(currentPosition, targetPosition);
        armMotor.setPower(power);
        armPID.setPID(parm, iarm, darm);

        if(armstate == ArmState.FRONT)
            updateArmState(gamepad);

        update(armstate);
    }

    public void update(ArmState state)
    {
        armstate = state;
        switch(armstate) {
            case FRONT:
                targetPosition = FRONT;
                break;
            case REAR:
                targetPosition = REAR;
                break;
        }
    }

    public void updateArmState(GamepadEx gamepad){
        if(gamepad.wasJustPressed(GamepadKeys.Button.B)){
            armstate = ArmState.REAR;
        }
    }
}
