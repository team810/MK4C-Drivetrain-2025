package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
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

        xController = new ProfiledPIDController(1,0,0,new TrapezoidProfile.Constraints(5,5));
        xController.setTolerance(.05,.5);
        xController.setGoal(new TrapezoidProfile.State(targetPose.getX(), targetXVelocity));
        yController = new ProfiledPIDController(1,0,0,new TrapezoidProfile.Constraints(5,5));
        yController.setTolerance(.05,.5);
        yController.setGoal(new TrapezoidProfile.State(targetPose.getY(), targetYVelocity));
        thetaController = new ProfiledPIDController(1,0,0,new TrapezoidProfile.Constraints(5,5));
        thetaController.setTolerance(.05,.5);
        thetaController.setGoal(new TrapezoidProfile.State(targetPose.getRotation().getRadians(), targetOmega));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Pose2d currentPose = DrivetrainSubsystem.getInstance().getPose();

        double yOutput = xController.calculate(currentPose.getX());
        double xOutput = yController.calculate(currentPose.getY());
        double omegaOutput = thetaController.calculate(currentPose.getRotation().getRadians());

//        Rotation2d heading = new Rotation2d(yOutput,yOutput);
//        double linearVelocity = Math.sqrt((xOutput * xOutput) + (yOutput * yOutput));
//        xOutput = linearVelocity * heading.getCos();
//        yOutput = linearVelocity * heading.getSin();

        ChassisSpeeds speeds = new ChassisSpeeds(xOutput,yOutput,omegaOutput);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, DrivetrainSubsystem.getInstance().getPose().getRotation());

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
