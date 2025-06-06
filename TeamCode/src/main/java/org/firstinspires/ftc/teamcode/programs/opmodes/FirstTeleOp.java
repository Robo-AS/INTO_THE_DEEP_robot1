package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@TeleOp(name = "FirstTeleOp", group = "OpModes")
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

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        //don't run this until telemetry works !!!!
//        robot.lift.loop();


        telemetry.addData("Current Position Right", robot.lift.rightSlider.getCurrentPosition());
        telemetry.addData("Current Position Left", robot.lift.leftSlider.getCurrentPosition());
        telemetry.addData("Target Position", robot.lift.getTargetPosition());
        telemetry.update();

    }
}
