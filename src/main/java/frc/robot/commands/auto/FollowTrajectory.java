package frc.robot.commands.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

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
        SwerveSample swerveSample;
        if (!timer.hasElapsed(trajectory.getTotalTime())) {
            swerveSample = trajectory.sampleAt(timer.get(), false).get();
        }else{
            swerveSample = trajectory.getFinalSample(false).get();
        }
        Pose2d currentPose = DrivetrainSubsystem.getInstance().getPose();

        double xOutput = xController.calculate(currentPose.getX(), swerveSample.getPose().getX()) + swerveSample.vx;
        double yOutput = yController.calculate(currentPose.getY(), swerveSample.getPose().getY()) + swerveSample.vy;
        double theta = thetaController.calculate(currentPose.getRotation().getRadians(), swerveSample.getPose().getRotation().getRadians()) + swerveSample.omega;

        double linearVelocity = Math.sqrt((xOutput * xOutput) + (yOutput * yOutput));
        Rotation2d heading = new Rotation2d(xOutput, yOutput);

        xOutput = heading.getCos() * linearVelocity;
        yOutput = heading.getSin() * linearVelocity;

        ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, theta);
        speeds.toRobotRelativeSpeeds(currentPose.getRotation());

        DrivetrainSubsystem.getInstance().setVelocityFOC(speeds);
        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTime());
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.off);
    }


}
