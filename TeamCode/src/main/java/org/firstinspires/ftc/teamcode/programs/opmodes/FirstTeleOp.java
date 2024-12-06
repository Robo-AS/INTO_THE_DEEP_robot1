package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

@TeleOp(name = "TeleOpDrive", group = "Linear Opmode")

public class FirstTeleOp extends LinearOpMode {

    GamepadEx driver;
    GamepadEx operator;

    Mecanum mecanum;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mecanum = new Mecanum(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            driver.readButtons();
            operator.readButtons();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            if(driver.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                mecanum.slowMotion(driver);
            }
            else{
                mecanum.teleop(driver,telemetry);
            }

        }

    }

}