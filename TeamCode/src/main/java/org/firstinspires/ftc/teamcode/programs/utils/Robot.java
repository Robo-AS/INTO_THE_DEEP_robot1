package org.firstinspires.ftc.teamcode.programs.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class Robot {
    private static Robot instance = null;
    private HardwareMap hardwareMap;

    //Lift Hardware
    public DcMotorEx leftSlider, rightSlider;
    public Lift lift;


    public static Robot getInstance() {//Why would i want to use this? It seems a bit useless
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }


    public void initializeHardware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

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

    }

    public void initialize() {
        lift.initialize();
    }

}
