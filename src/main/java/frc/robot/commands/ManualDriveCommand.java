package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.Superstructure;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem.ControlMethods;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 *  This command is always running while in teleop
 */
public class ManualDriveCommand extends Command {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter omegaLimiter;

    private final SlewRateLimiter xLimiterGP;
    private final SlewRateLimiter yLimiterGP;
    private final PIDController xController;
    private final PIDController yController;

    private final DoubleSupplier driveXVelocity;
    private final DoubleSupplier driveYVelocity;
    private final DoubleSupplier driveOmega;

    private final BooleanSupplier gpLock;
    private final BooleanSupplier yawLock;

    private boolean hasBeenZero = true;
    private Rotation2d lockedHeading = new Rotation2d();

    public enum yawControl{
        omega,
        target,
    }

    private yawControl control;

    public ManualDriveCommand() {
        xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION);
        omegaLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);

        xController = new PIDController(.07, 0, 0);
        yController = new PIDController(.07, 0, 0);
        xController.setTolerance(.2);
        yController.setTolerance(.2);

        xLimiterGP = new SlewRateLimiter(5);
        yLimiterGP = new SlewRateLimiter(5);

        control = yawControl.omega;

        driveXVelocity = IO.getJoystickValue(Controls.driveXVelocity);
        driveYVelocity = IO.getJoystickValue(Controls.driveYVelocity);
        driveOmega = IO.getJoystickValue(Controls.driveOmega);

        gpLock = IO.getButtonValue(Controls.gpLock);
        yawLock = IO.getButtonValue(Controls.yawLock);
    }

    @Override
    public void execute() {
        if (!gpLock.getAsBoolean()) {

            if (yawLock.getAsBoolean()) {
                control = yawControl.target;
            }else{
                control = yawControl.omega;
            }

            double verticalVelocity;
            double horizontalVelocity;
            double invert = 1;

            if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Red)
            {
                invert = invert * -1;
            }
            horizontalVelocity = -driveYVelocity.getAsDouble();
            verticalVelocity = -driveXVelocity.getAsDouble();

            horizontalVelocity = horizontalVelocity * invert;
            verticalVelocity = verticalVelocity * invert;

            horizontalVelocity = MathUtil.applyDeadband(horizontalVelocity, .1);
            verticalVelocity = MathUtil.applyDeadband(verticalVelocity, .1);

            verticalVelocity = verticalVelocity * DrivetrainConstants.MAX_VELOCITY;
            horizontalVelocity = horizontalVelocity * DrivetrainConstants.MAX_VELOCITY;

            verticalVelocity = xLimiter.calculate(verticalVelocity);
            horizontalVelocity = yLimiter.calculate(horizontalVelocity);

            switch (control)
            {
                case omega -> {
                    double omegaVelocity;

                    omegaVelocity = -driveOmega.getAsDouble();
                    omegaVelocity = MathUtil.applyDeadband(omegaVelocity, .05);

                    if ((omegaVelocity == 0 && !hasBeenZero) | (Math.abs(DrivetrainSubsystem.getInstance().getPose().getRotation().minus(lockedHeading).getDegrees()) > 2)) {
                        hasBeenZero = true;
                        lockedHeading = DrivetrainSubsystem.getInstance().getPose().getRotation();
                    }else if (omegaVelocity != 0) {
                        hasBeenZero = false;
                    }

                    if (omegaVelocity == 0 && Math.abs(DrivetrainSubsystem.getInstance().getRate().in(Units.DegreesPerSecond)) < 25) {
                        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityThetaControlFOC);
                        DrivetrainSubsystem.getInstance().setVelocityThetaControlFOC(horizontalVelocity,verticalVelocity, lockedHeading,true);
                    }else{
                        omegaVelocity = omegaVelocity * DrivetrainConstants.MAX_ANGULAR_VELOCITY;
                        omegaVelocity = omegaLimiter.calculate(omegaVelocity);

                        ChassisSpeeds targetSpeeds;
                        targetSpeeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omegaVelocity);
                        targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, DrivetrainSubsystem.getInstance().getPose().getRotation());
                        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
                        DrivetrainSubsystem.getInstance().setVelocityFOC(targetSpeeds);
                    }
                }
                case target -> {
                
                    // This will be alliance specific points
                    ArrayList<Pose2d> landMarks = new ArrayList<>();
                    landMarks.add(new Pose2d(0,0,new Rotation2d()));
                    landMarks.add(new Pose2d(0,3.048,new Rotation2d()));
                    // landMarks.add(new Pose2d(2.67,1.17,new Rotation2d()));

                    Pose2d currentPose;
                    currentPose = DrivetrainSubsystem.getInstance().getPose();
                    Pose2d closetPose = currentPose.nearest(landMarks);

                    double xDistance = currentPose.getX() - closetPose.getX();
                    double yDistance = currentPose.getY() - closetPose.getY();
                    double theta = Math.atan2(-yDistance,-xDistance);

                    DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityThetaControlFOC);
                    DrivetrainSubsystem.getInstance().setVelocityThetaControlFOC(horizontalVelocity,verticalVelocity,Rotation2d.fromRadians(theta),false);
                }
            }
        }else{
            if (VisionSubsystem.getInstance().isTargetDetected()) {
                double currentX = VisionSubsystem.getInstance().getTargetX();
                double currentY = VisionSubsystem.getInstance().getTargetY();

                double xOutput = -xController.calculate(currentX, 0);
                double yOutput = yController.calculate(currentY, 0);

                xOutput = MathUtil.clamp(xOutput, -3,3);
                yOutput = MathUtil.clamp(yOutput, -3,3);

                xOutput = xLimiterGP.calculate(xOutput);
                yOutput = yLimiterGP.calculate(yOutput);

                DrivetrainSubsystem.getInstance().setVelocityRR(new ChassisSpeeds(yOutput, xOutput,0));
                DrivetrainSubsystem.getInstance().setControlMode(ControlMethods.VelocityRR);
            }

        }

    }


    @Override
    public boolean isFinished() {
        return !RobotState.isTeleop();
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setVelocityFOC(new ChassisSpeeds());
        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.off);
    }
}
