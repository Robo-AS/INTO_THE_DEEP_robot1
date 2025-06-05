package org.firstinspires.ftc.teamcode.programs.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.commandbase.intake.PickUpBackToInit;
import org.firstinspires.ftc.teamcode.programs.commandbase.outtake.FromHighBasketBackToIdle;
import org.firstinspires.ftc.teamcode.programs.commandbase.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.programs.commandbase.outtake.HighChamberBackToIdle;
import org.firstinspires.ftc.teamcode.programs.subsystems.Camera;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "RedSampleTeleOp", group = "OpModes")
public class RedSampleTeleOp extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap);
        robot.initialize();

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new FromHighBasketBackToIdle());

        gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(new IntakeClaw());

        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new PickUpBackToInit());

        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new HighChamberBackToIdle());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        robot.getInstanceLift().loop();
        robot.getInstanceArm().loop();
        robot.mecanum.loop(gamepadEx);
        robot.getInstanceClaw().loop();
        robot.camera.loop();

        robot.camera.setDetectionMode(Camera.Detection.RED_YELLOW);

        telemetry.addData("Current Position", robot.rightSlider.getCurrentPosition());
        telemetry.addData("Target Position", robot.getInstanceLift().getTargetPosition());
        telemetry.update();

    }
}