package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Camera;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

@TeleOp(name = "TeleOpDrive", group = "Linear Opmode")

public class FirstTeleOp extends LinearOpMode {

    GamepadEx driver;
    GamepadEx operator;

    Mecanum mecanum;
    Lift lift;
    Arm arm;

    private Camera camera;
    private ElapsedTime runtime = new ElapsedTime();
    double loopTime = 0;
    @Override
    public void runOpMode() {
        camera = new Camera();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mecanum = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);

        waitForStart();
        lift.initializeLift(hardwareMap);
        arm.initializeHardware(hardwareMap);
        runtime.reset();

        while (opModeIsActive()) {
            driver.readButtons();
            operator.readButtons();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            telemetry.addData("Status: ", "Python processing");

            if(driver.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                mecanum.slowMotion(driver);
            }
            else {
                mecanum.teleop(driver, telemetry);
            }

            lift.loop(driver);
            arm.loop(driver);

            while(camera.isBlueObjectDetected())
            {
                mecanum.moveForwardForWebcamTest();
            }
            double loop = System.nanoTime();

            telemetry.addData("hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }

}