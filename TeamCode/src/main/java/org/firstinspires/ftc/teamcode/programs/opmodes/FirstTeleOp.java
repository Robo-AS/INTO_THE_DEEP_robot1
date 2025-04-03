package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.commandbase.outtake.FromHighBasketBackToIdle;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "TeleOp", group = "OpModes")
public class FirstTeleOp extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap);
        robot.initialize();

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new FromHighBasketBackToIdle());

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        robot.lift.loop();
        robot.arm.loop();
        robot.mecanum.loop(gamepadEx);

        telemetry.addData("Current Position", robot.rightSlider.getCurrentPosition());
        telemetry.addData("Target Position", robot.lift.getTargetPosition());
        telemetry.update();

    }
}
