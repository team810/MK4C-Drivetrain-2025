package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class Interlope extends Command {
    private final Pose2d targetPose;

    private final double targetXVelocity;
    private final double targetYVelocity;
    private final double targetOmega;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    public Interlope(Pose2d targetPose, double targetXVelocity, double targetYVelocity, double omega) {
        this.targetPose = targetPose;
        this.targetXVelocity = targetXVelocity;
        this.targetYVelocity = targetYVelocity;
        this.targetOmega = omega;

        xController = new ProfiledPIDController(0,0,0,new TrapezoidProfile.Constraints(5,5));
        xController.setTolerance(.05,.5);
        yController = new ProfiledPIDController(0,0,0,new TrapezoidProfile.Constraints(5,5));
        yController.setTolerance(.05,.5);
        thetaController = new ProfiledPIDController(0,0,0,new TrapezoidProfile.Constraints(5,5));
        thetaController.setTolerance(.05,.5);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Pose2d currentPose = DrivetrainSubsystem.getInstance().getPose();

        double xOutput = xController.calculate(currentPose.getX(), new TrapezoidProfile.State(targetPose.getX(), targetXVelocity));
        double yOutput = yController.calculate(currentPose.getY(), new TrapezoidProfile.State(targetPose.getY(), targetYVelocity));
        double omegaOutput = thetaController.calculate(currentPose.getRotation().getRadians(), new TrapezoidProfile.State(targetPose.getRotation().getRadians(), targetOmega));

        Rotation2d heading = new Rotation2d(xOutput,yOutput);
        double linearVelocity = Math.sqrt((xOutput * xOutput) + (yOutput * yOutput));
        xOutput = linearVelocity * Math.cos(heading.getRadians());
        yOutput = linearVelocity * Math.sin(heading.getRadians());

        ChassisSpeeds speeds = new ChassisSpeeds(xOutput,yOutput,omegaOutput);
        speeds.toRobotRelativeSpeeds(DrivetrainSubsystem.getInstance().getPose().getRotation());

        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
        DrivetrainSubsystem.getInstance().setVelocityFOC(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.off);
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
}
