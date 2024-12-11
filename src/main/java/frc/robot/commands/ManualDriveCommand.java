package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Superstructure;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.ArrayList;

/**
 *  This command is always running while in teleop
 */
public class ManualDriveCommand extends Command {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter omegaLimiter;

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

        control = yawControl.omega;

        addRequirements(DrivetrainSubsystem.getInstance());
    }

    @Override
    public void execute() {
        if (IO.getButtonValue(Controls.yawLock).getAsBoolean())
        {
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

        horizontalVelocity = -IO.getJoystickValue(Controls.driveYVelocity).get();
        verticalVelocity = -IO.getJoystickValue(Controls.driveXVelocity).get();

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

                omegaVelocity = -IO.getJoystickValue(Controls.driveOmega).get(); // CCW position so left positive is good
                omegaVelocity = MathUtil.applyDeadband(omegaVelocity, .05);

                if ((omegaVelocity == 0 && !hasBeenZero) | (Math.abs(DrivetrainSubsystem.getInstance().getPose().getRotation().minus(lockedHeading).getDegrees()) > 2)) {
                    hasBeenZero = true;
                    lockedHeading = DrivetrainSubsystem.getInstance().getPose().getRotation();
                }else if (omegaVelocity != 0) {
                    hasBeenZero = false;
                }

                if (omegaVelocity == 0 && Math.abs(DrivetrainSubsystem.getInstance().getRate().in(Units.DegreesPerSecond)) < 10) {
                    DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityThetaControlFOC);
                    DrivetrainSubsystem.getInstance().setVelocityThetaControlFOC(horizontalVelocity,verticalVelocity, lockedHeading,true);
                }else{
                    omegaVelocity = omegaVelocity * DrivetrainConstants.MAX_ANGULAR_VELOCITY;
                    omegaVelocity = omegaLimiter.calculate(omegaVelocity);

                    ChassisSpeeds targetSpeeds;
                    targetSpeeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omegaVelocity);
                    targetSpeeds.toRobotRelativeSpeeds(DrivetrainSubsystem.getInstance().getPose().getRotation());
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
