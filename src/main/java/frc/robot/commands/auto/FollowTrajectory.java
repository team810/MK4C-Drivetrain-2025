package frc.robot.commands.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowTrajectory extends Command {
    private final Timer timer;
    private final Trajectory<SwerveSample> trajectory;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public FollowTrajectory(Trajectory<SwerveSample> trajectory) {
        timer = new Timer();

        xController = new PIDController(0,0,0);
        yController = new PIDController(0,0,0);
        thetaController = new PIDController(0,0,0);

        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
