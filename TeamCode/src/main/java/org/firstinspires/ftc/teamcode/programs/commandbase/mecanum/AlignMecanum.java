package org.firstinspires.ftc.teamcode.programs.commandbase.mecanum;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.programs.subsystems.Camera;
import org.firstinspires.ftc.teamcode.programs.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.programs.utils.SquIDController;

public class AlignMecanum extends CommandBase {

    private final Camera camera;
    private final Mecanum mecanum;

    private final SquIDController translationalController = new SquIDController(0.1);
    public AlignMecanum(Camera camera, Mecanum mecanum) {
        this.camera = camera;
        this.mecanum = mecanum;

        addRequirements(mecanum);
    }

    @Override
    public void execute() {
        double xEffort = translationalController.calculate(0.1, 0, camera.getTx()-0.155); //applying the offset, it was actually 15.5 cm
        double yEffort = -translationalController.calculate(0.1, 0, camera.getTy());

        mecanum.drive(
                () -> xEffort,
                () -> yEffort
        );
    }

    @Override
    public void end(boolean interrupted) {
        mecanum.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(camera.getTx() - 0.155) < 0.02 && Math.abs(camera.getTy()) < 0.02;
    }

}
