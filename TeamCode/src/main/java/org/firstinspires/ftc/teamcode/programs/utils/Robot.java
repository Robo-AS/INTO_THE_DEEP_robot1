package org.firstinspires.ftc.teamcode.programs.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;

import java.util.Arrays;
import java.util.List;

public class Robot {
    private static Robot instance = null;
    private static Lift instanceLift = null;
    private static Arm instanceArm = null;
    private HardwareMap hardwareMap;

    //Lift Hardware
    public DcMotorEx leftSlider, rightSlider;
    public Lift lift;

    // Arm Hardware

    public DcMotorEx armMotor;
    public Arm arm;

    //Mecanum hardware

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public List<DcMotorEx> motors;

    public Mecanum mecanum;


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
        rightSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift = new Lift();

        //arm

        armMotor = hardwareMap.get(DcMotorEx.class, "Circular");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = new Arm();

    }

    public void initialize() {
        arm.initialize();
        lift.initialize();
    }

    public static Lift getInstanceLift(){
        if (instanceLift == null) {
            instanceLift = new Lift();
        }
        return instanceLift;
    }
    public static Arm getInstanceArm(){
        if (instanceArm == null) {
            instanceArm = new Arm();
        }
        return instanceArm;
    }


}
