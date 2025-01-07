package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class GoToPose extends Command {
    private final Pose2d targetPose;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    private Pose2d currentPose;

    public GoToPose(Pose2d targetPose) {
        xController = new PIDController(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION * .4,0,0); // kp is in meters per second squared
        yController = new PIDController(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION * .4,0,0); // kp is in meters per second squared
        thetaController = new PIDController(DrivetrainConstants.MAX_ANGULAR_ACCELERATION * .4,0,0); // kp is in radians per second squared

        xController.setTolerance(.05);
        yController.setTolerance(.05);
        thetaController.setTolerance(Math.toRadians(1));

        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        currentPose = DrivetrainSubsystem.getInstance().getPose();
    }

    @Override
    public void execute() {
        double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
        double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians(),targetPose.getRotation().getRadians());

        Rotation2d heading = new Rotation2d(xOutput,yOutput);
        double linearVelocity = Math.sqrt((xOutput * xOutput) + (yOutput * yOutput));
        linearVelocity = MathUtil.clamp(
                linearVelocity,
                -DrivetrainConstants.MAX_VELOCITY,
                DrivetrainConstants.MAX_VELOCITY
        );
        xOutput = heading.getCos() * linearVelocity;
        yOutput = heading.getSin() * linearVelocity;

        thetaOutput = MathUtil.clamp(thetaOutput, -6, 6); // clamped from -6 to 6
        ChassisSpeeds speeds = new ChassisSpeeds(xOutput,yOutput,thetaOutput);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, DrivetrainSubsystem.getInstance().getPose().getRotation());

        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
        DrivetrainSubsystem.getInstance().setVelocityFOC(speeds);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

}
