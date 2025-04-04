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
    public Lift lift;

    private Robot(){
        lift = Lift.getInstance();
    }


    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }


    public void initializeHardware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        lift.initializeHardware(hardwareMap);


    }

    public void initialize() {
        lift.initialize();
    }

}
