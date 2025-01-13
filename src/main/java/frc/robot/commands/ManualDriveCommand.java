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
import frc.robot.FiledConstants;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.Superstructure;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 *  This command is always running while in teleop
 */
public class ManualDriveCommand extends Command {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter omegaLimiter;

    private final PIDController xAlignController;
    private final PIDController yAlignController;
    private final PIDController omegaAlignController;

    private final DoubleSupplier driveXVelocity;
    private final DoubleSupplier driveYVelocity;
    private final DoubleSupplier driveOmega;

    private final BooleanSupplier leftAlign;
    private final BooleanSupplier rightAlign;

    private final BooleanSupplier leftSource;
    private final BooleanSupplier rightSource;

    private boolean hasBeenZero = true;
    private Rotation2d lockedHeading = new Rotation2d();

    private final ArrayList<Pose2d> reefSections = new ArrayList<>();

    private final Pose2d F;
    private final Pose2d F_LEFT;
    private final Pose2d F_RIGHT;

    private final Pose2d FL;
    private final Pose2d FL_LEFT;
    private final Pose2d FL_RIGHT;

    private final Pose2d BL;
    private final Pose2d BL_LEFT;
    private final Pose2d BL_RIGHT;

    private final Pose2d B;
    private final Pose2d B_LEFT;
    private final Pose2d B_RIGHT;

    private final Pose2d BR;
    private final Pose2d BR_LEFT;
    private final Pose2d BR_RIGHT;

    private final Pose2d FR;
    private final Pose2d FR_LEFT;
    private final Pose2d FR_RIGHT;

    private Pose2d targetPose = new Pose2d();
    private boolean alignLastTick = false; // Was the code in align mode last tick

    public ManualDriveCommand() {
        xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION * 100);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION * 100);
        omegaLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION * 100);

        xAlignController = new PIDController(7, 0, 0);
        yAlignController = new PIDController(7, 0, 0);
        xAlignController.setTolerance(.02);
        yAlignController.setTolerance(.02);
        omegaAlignController = new PIDController(4, 0, 0);
        omegaAlignController.enableContinuousInput(-Math.PI, Math.PI);
        omegaAlignController.setTolerance(Math.toRadians(1));

        driveXVelocity = IO.getJoystickValue(Controls.driveXVelocity);
        driveYVelocity = IO.getJoystickValue(Controls.driveYVelocity);
        driveOmega = IO.getJoystickValue(Controls.driveOmega);

        leftAlign = IO.getButtonValue(Controls.leftAlign);
        rightAlign = IO.getButtonValue(Controls.rightAlign);

        leftSource = IO.getButtonValue(Controls.leftSource);
        rightSource = IO.getButtonValue(Controls.rightSource);

        if (Superstructure.getInstance().getAlliance().equals(DriverStation.Alliance.Blue)) {
            reefSections.add(FiledConstants.BlueReef.F);
            reefSections.add(FiledConstants.BlueReef.FL);
            reefSections.add(FiledConstants.BlueReef.BL);
            reefSections.add(FiledConstants.BlueReef.B);
            reefSections.add(FiledConstants.BlueReef.BR);
            reefSections.add(FiledConstants.BlueReef.FR);

            F = FiledConstants.BlueReef.F;
            F_LEFT = FiledConstants.BlueReef.F_LEFT;
            F_RIGHT = FiledConstants.BlueReef.F_RIGHT;

            FL = FiledConstants.BlueReef.FL;
            FL_LEFT = FiledConstants.BlueReef.FL_LEFT;
            FL_RIGHT = FiledConstants.BlueReef.FL_RIGHT;

            BL = FiledConstants.BlueReef.BL;
            BL_LEFT = FiledConstants.BlueReef.BL_LEFT;
            BL_RIGHT = FiledConstants.BlueReef.BL_RIGHT;

            B = FiledConstants.BlueReef.B;
            B_LEFT = FiledConstants.BlueReef.B_LEFT;
            B_RIGHT = FiledConstants.BlueReef.B_RIGHT;

            BR = FiledConstants.BlueReef.BR;
            BR_LEFT = FiledConstants.BlueReef.BR_LEFT;
            BR_RIGHT = FiledConstants.BlueReef.BR_RIGHT;

            FR = FiledConstants.BlueReef.FR;
            FR_LEFT = FiledConstants.BlueReef.FR_LEFT;
            FR_RIGHT = FiledConstants.BlueReef.FR_RIGHT;
        }else{
            reefSections.add(FiledConstants.RedReef.F);
            reefSections.add(FiledConstants.RedReef.FL);
            reefSections.add(FiledConstants.RedReef.BL);
            reefSections.add(FiledConstants.RedReef.B);
            reefSections.add(FiledConstants.RedReef.BR);
            reefSections.add(FiledConstants.RedReef.FR);

            F = FiledConstants.RedReef.F;
            F_LEFT = FiledConstants.RedReef.F_LEFT;
            F_RIGHT = FiledConstants.RedReef.F_RIGHT;

            FL = FiledConstants.RedReef.FL;
            FL_LEFT = FiledConstants.RedReef.FL_LEFT;
            FL_RIGHT = FiledConstants.RedReef.FL_RIGHT;

            BL = FiledConstants.RedReef.BL;
            BL_LEFT = FiledConstants.RedReef.BL_LEFT;
            BL_RIGHT = FiledConstants.RedReef.BL_RIGHT;

            B = FiledConstants.RedReef.B;
            B_LEFT = FiledConstants.RedReef.B_LEFT;
            B_RIGHT = FiledConstants.RedReef.B_RIGHT;

            BR = FiledConstants.RedReef.BR;
            BR_LEFT = FiledConstants.RedReef.BR_LEFT;
            BR_RIGHT = FiledConstants.RedReef.BR_RIGHT;

            FR = FiledConstants.RedReef.FR;
            FR_LEFT = FiledConstants.RedReef.FR_LEFT;
            FR_RIGHT = FiledConstants.RedReef.FR_RIGHT;
        }
    }

//    @Logged (name = "Target Pose")
//    public Pose2d getTargetPose() {
//        if (targetPose == null)
//        {
//            return new Pose2d();
//        }else{
//            return targetPose;
//        }
//    }
    @Override
    public void execute() {
        boolean left = leftAlign.getAsBoolean();
        boolean right = rightAlign.getAsBoolean();

        boolean leftSourceB = leftSource.getAsBoolean();
        boolean rightSourceB = rightSource.getAsBoolean();

        if (!(left || right || leftSourceB || rightSourceB)) {

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

            double omegaVelocity;

            omegaVelocity = -driveOmega.getAsDouble();
            omegaVelocity = MathUtil.applyDeadband(omegaVelocity, .05);

            if ((omegaVelocity == 0 && !hasBeenZero) | (Math.abs(DrivetrainSubsystem.getInstance().getPose().getRotation().minus(lockedHeading).getDegrees()) > 2)) {
                hasBeenZero = true;
                lockedHeading = DrivetrainSubsystem.getInstance().getPose().getRotation();
            }else if (omegaVelocity != 0) {
                hasBeenZero = false;
            }

            if (omegaVelocity == 0 && Math.abs(DrivetrainSubsystem.getInstance().getRate().in(Units.DegreesPerSecond)) < 45) {
                DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityThetaControlFOC);
                DrivetrainSubsystem.getInstance().setVelocityThetaControlFOC(horizontalVelocity,verticalVelocity, lockedHeading,true);
            }else{
                omegaVelocity = omegaVelocity * DrivetrainConstants.MAX_ANGULAR_VELOCITY;
                omegaVelocity = omegaLimiter.calculate(omegaVelocity);

                ChassisSpeeds targetSpeeds;
                targetSpeeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omegaVelocity);
//                targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, DrivetrainSubsystem.getInstance().getPose().getRotation());
                DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
                DrivetrainSubsystem.getInstance().setVelocityFOC(targetSpeeds);
            }
            alignLastTick = false;
        } else if (left || right) {
            Pose2d currentPose = DrivetrainSubsystem.getInstance().getPose();
            if (!alignLastTick) {
                targetPose = currentPose.nearest(reefSections);
                if (targetPose == F) {
                    System.out.println("Front");
                    if (left) {
                        targetPose = F_LEFT;
                    } else {
                        targetPose = F_RIGHT;
                    }
                }else if (targetPose == FL) {
                    System.out.println("FrontLeft");
                    if (left) {
                        targetPose = FL_LEFT;
                    }else {
                        targetPose = FL_RIGHT;
                    }
                }else if (targetPose == BL) {
                    System.out.println("Back Left");
                    if (left) {
                        targetPose = BL_LEFT;
                    }else {
                        targetPose = BL_RIGHT;
                    }
                }else if (targetPose == B) {
                    if (left) {
                        targetPose = B_LEFT;
                    }else {
                        targetPose = B_RIGHT;
                    }
                }else if (targetPose == BR) {
                    if (left) {
                        targetPose = BR_LEFT;
                    }else{
                        targetPose = BR_RIGHT;
                    }
                }else if (targetPose == FR) {
                    if (left) {
                        targetPose = FR_LEFT;
                    }else{
                        targetPose = FR_RIGHT;
                    }
                }else{

                }
                alignLastTick = true;
            }

            double xOutput = xAlignController.calculate(currentPose.getX(), targetPose.getX());
            double yOutput = yAlignController.calculate(currentPose.getY(), targetPose.getY());
            double omegaOutput = omegaAlignController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            xOutput = MathUtil.clamp(xOutput, -3, 3);
            yOutput = MathUtil.clamp(yOutput, -3, 3);
            omegaOutput = MathUtil.clamp(omegaOutput, -6, 6);

            ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, omegaOutput);
//            ChassisSpeeds.fromFieldRelativeSpeeds(speeds,currentPose.getRotation());
            DrivetrainSubsystem.getInstance().setVelocityFOC(speeds);
            DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
        } else if (leftSourceB || rightSourceB) {
            Pose2d currentPose = DrivetrainSubsystem.getInstance().getPose();
            Pose2d targetPose = new Pose2d();
            if (leftSourceB) {
                targetPose = new Pose2d(1.6,7.3, new Rotation2d(2.223));
            }else if (rightSourceB){
                targetPose = new Pose2d(1.6, .719, new Rotation2d(-2.223));
            }

            double xOutput = xAlignController.calculate(currentPose.getX(), targetPose.getX());
            double yOutput = yAlignController.calculate(currentPose.getY(), targetPose.getY());
            double omegaOutput = omegaAlignController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            xOutput = MathUtil.clamp(xOutput, -4.5, 4.5);
            yOutput = MathUtil.clamp(yOutput, -4.5, 4.5);
            omegaOutput = MathUtil.clamp(omegaOutput, -6, 6);

            ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, omegaOutput);
//            ChassisSpeeds.fromFieldRelativeSpeeds(speeds,currentPose.getRotation());
            DrivetrainSubsystem.getInstance().setVelocityFOC(speeds);
            DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
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
