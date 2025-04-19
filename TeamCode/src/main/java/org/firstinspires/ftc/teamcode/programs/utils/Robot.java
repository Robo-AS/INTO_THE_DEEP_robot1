package org.firstinspires.ftc.teamcode.programs.utils;

import com.qualcomm.ftccommon.CommandList;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Camera;
import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;

import java.util.Arrays;
import java.util.List;

public class Robot {
    private static Robot instance = null;
    private static HardwareMap hardwareMap;

    //Lift Hardware
    public DcMotorEx leftSlider, rightSlider;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public List<DcMotorEx> motors;

    // Arm Hardware

    public DcMotorEx armMotor;
    private static Arm arm = null;

    public Mecanum mecanum;
    public Camera camera;

    private static Claw claw = null;

    private static Lift lift = null;

    public Servo servo0, servo1, servo2, servo3, servo4;

    //Mecanum hardware

    public static Robot getInstance() {//Why would i want to use this? It seems a bit useless
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }


    public void initializeHardware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        //mecanum

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        mecanum = new Mecanum();
        //lift

        leftSlider = hardwareMap.get(DcMotorEx.class, "leftSlider");
        rightSlider = hardwareMap.get(DcMotorEx.class, "rightSlider");

        leftSlider.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlider.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //arm

        armMotor = hardwareMap.get(DcMotorEx.class, "Circular");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

       // Servo extendServo = hardwareMap.get(Servo.class, "extendServo");

        //Claw

        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");

        servo3.setDirection(Servo.Direction.REVERSE);
        servo1.setDirection(Servo.Direction.REVERSE);

        camera = new Camera();
    }

    public void initialize() {
        Robot.getInstance().getInstanceArm().initialize();
        Robot.getInstance().getInstanceLift().initialize();
        Robot.getInstance().getInstanceClaw().initialize();
        Robot.getInstance().camera.init();
    }

    public static Lift getInstanceLift(){
        if (lift == null) {
            lift = new Lift();
        }
        return lift;
    }
    public static Arm getInstanceArm(){
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

    public static Claw getInstanceClaw(){
        if (claw == null) {
            claw = new Claw();
        }
        return claw;
    }

    public static HardwareMap getInstanceHardwareMap(){
        return hardwareMap;
    }
}
