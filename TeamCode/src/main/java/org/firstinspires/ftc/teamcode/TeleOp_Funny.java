package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Chasis_Mecha.powerReduction;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Chasis_Mecha;
@TeleOp(name = "TeleOpDrive", group= "Linear Opmode")
public class TeleOp_Funny extends LinearOpMode {

    GamepadEx driver;
    Chasis_Mecha body;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        driver = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        body = new Chasis_Mecha(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            driver.readButtons();
            if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                body.Slow_Motion(driver, telemetry);
            }
            else{
                body.teleop(driver, telemetry);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}