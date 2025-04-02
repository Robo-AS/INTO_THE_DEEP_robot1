package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.programs.subsystems.Camera;
import org.firstinspires.ftc.teamcode.programs.subsystems.Claw;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

@TeleOp(name = "TeleOpDrive", group = "Linear Opmode")

public class FirstTeleOp extends LinearOpMode {
    GamepadEx driver;
    GamepadEx operator;

    public Mecanum mecanum;
    Lift lift;
    Arm arm;
    Camera camera;

    Claw claw;
    private ElapsedTime runtime = new ElapsedTime();
    double loopTime = 0;
    @Override
    public void runOpMode()
    {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        mecanum = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        camera = new Camera(hardwareMap, telemetry);
        claw = new Claw(hardwareMap);

        waitForStart();
        lift.initializeLift(hardwareMap);
        arm.initializeHardware(hardwareMap);
        runtime.reset();

        while(opModeIsActive()) {
            driver.readButtons();
            operator.readButtons();


            if(driver.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                mecanum.slowMotion(driver);
            }
            else {
                mecanum.teleop(driver, telemetry);
            }

            lift.loop(driver);

            if(driver.wasJustPressed(GamepadKeys.Button.A)){
                lift.initialize();
            }

            arm.loop(driver);

            if(driver.wasJustPressed(GamepadKeys.Button.X)){
                 arm.initialize();
            }

            //camera.cameraLoop(mecanum, claw);

            claw.loop(driver);
        }
    }

}