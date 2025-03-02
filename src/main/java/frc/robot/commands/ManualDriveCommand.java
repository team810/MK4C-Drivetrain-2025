package frc.robot.commands;

import choreo.util.ChoreoAllianceFlipUtil;
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
import frc.robot.FieldConstants;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.Superstructure;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 *  This command is always running while in teleop
 */
public class ManualDriveCommand extends Command {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final SlewRateLimiter velocityLimiter;

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

    private final Pose2d LEFT_SOURCE;
    private final Pose2d RIGHT_SOURCE;

    private Pose2d targetPose = new Pose2d();
    private boolean alignLastTick = false; // Was the code in align mode last tick

    private double invert = 1;

    public ManualDriveCommand() {
        xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION); // Fix to match the actual constants of the drivetrain
        yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION);

        velocityLimiter = new SlewRateLimiter(8);

        xAlignController = new PIDController(3.5, 0, 0);
        yAlignController = new PIDController(3.5, 0, 0);
        xAlignController.setTolerance(.02);
        yAlignController.setTolerance(.02);
        omegaAlignController = new PIDController(7, 0, 0);
        omegaAlignController.enableContinuousInput(-Math.PI, Math.PI);
        omegaAlignController.setTolerance(Math.toRadians(.2));

        driveXVelocity = IO.getJoystickValue(Controls.driveXVelocity);
        driveYVelocity = IO.getJoystickValue(Controls.driveYVelocity);
        driveOmega = IO.getJoystickValue(Controls.driveOmega);

        leftAlign = IO.getButtonValue(Controls.leftAlign);
        rightAlign = IO.getButtonValue(Controls.rightAlign);

        leftSource = IO.getButtonValue(Controls.leftSource);
        rightSource = IO.getButtonValue(Controls.rightSource);

        if (Superstructure.getInstance().getAlliance().equals(DriverStation.Alliance.Red)) {
            F = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.F);
            F_LEFT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.F_LEFT);
            F_RIGHT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.F_RIGHT);

            FL = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.FL);
            FL_LEFT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.FL_LEFT);
            FL_RIGHT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.FL_RIGHT);

            BL = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.BL);
            BL_LEFT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.BL_LEFT);
            BL_RIGHT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.BL_RIGHT);

            B = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.B);
            B_LEFT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.B_LEFT);
            B_RIGHT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.B_RIGHT);

            BR = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.BR);
            BR_LEFT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.BR_LEFT);
            BR_RIGHT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.BR_RIGHT);

            FR = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.FR);
            FR_LEFT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.FR_LEFT);
            FR_RIGHT = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.FR_RIGHT);

            LEFT_SOURCE = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.LEFT_SOURCE);
            RIGHT_SOURCE = ChoreoAllianceFlipUtil.flip(FieldConstants.BlueReef.RIGHT_SOURCE);

            reefSections.add(F);
            reefSections.add(FL);
            reefSections.add(BL);
            reefSections.add(B);
            reefSections.add(BR);
            reefSections.add(FR);
        }else{
            F = FieldConstants.BlueReef.F;
            F_LEFT = FieldConstants.BlueReef.F_LEFT;
            F_RIGHT = FieldConstants.BlueReef.F_RIGHT;

            FL = FieldConstants.BlueReef.FL;
            FL_LEFT = FieldConstants.BlueReef.FL_LEFT;
            FL_RIGHT = FieldConstants.BlueReef.FL_RIGHT;

            BL = FieldConstants.BlueReef.BL;
            BL_LEFT = FieldConstants.BlueReef.BL_LEFT;
            BL_RIGHT = FieldConstants.BlueReef.BL_RIGHT;

            B = FieldConstants.BlueReef.B;
            B_LEFT = FieldConstants.BlueReef.B_LEFT;
            B_RIGHT = FieldConstants.BlueReef.B_RIGHT;

            BR = FieldConstants.BlueReef.BR;
            BR_LEFT = FieldConstants.BlueReef.BR_LEFT;
            BR_RIGHT = FieldConstants.BlueReef.BR_RIGHT;

            FR = FieldConstants.BlueReef.FR;
            FR_LEFT = FieldConstants.BlueReef.FR_LEFT;
            FR_RIGHT = FieldConstants.BlueReef.FR_RIGHT;

            LEFT_SOURCE = FieldConstants.BlueReef.LEFT_SOURCE;
            RIGHT_SOURCE = FieldConstants.BlueReef.RIGHT_SOURCE;

            reefSections.add(FieldConstants.BlueReef.F);
            reefSections.add(FieldConstants.BlueReef.FL);
            reefSections.add(FieldConstants.BlueReef.BL);
            reefSections.add(FieldConstants.BlueReef.B);
            reefSections.add(FieldConstants.BlueReef.BR);
            reefSections.add(FieldConstants.BlueReef.FR);
        }


        if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Red)
        {
            invert = invert * -1;
        }
    }
    @Override
    public void execute() {
        boolean left = leftAlign.getAsBoolean();
        boolean right = rightAlign.getAsBoolean();

        boolean leftSourceB = leftSource.getAsBoolean();
        boolean rightSourceB = rightSource.getAsBoolean();

        if (!(left || right || leftSourceB || rightSourceB)) {

            double verticalVelocity;
            double horizontalVelocity;

            horizontalVelocity = -driveYVelocity.getAsDouble();
            verticalVelocity = -driveXVelocity.getAsDouble();

            horizontalVelocity = horizontalVelocity * invert;
            verticalVelocity = verticalVelocity * invert;

            horizontalVelocity = MathUtil.applyDeadband(horizontalVelocity, .06);
            verticalVelocity = MathUtil.applyDeadband(verticalVelocity, .06);

            verticalVelocity = verticalVelocity * DrivetrainConstants.MAX_VELOCITY;
            horizontalVelocity = horizontalVelocity * DrivetrainConstants.MAX_VELOCITY;

            double omegaVelocity;

            omegaVelocity = -driveOmega.getAsDouble();
            omegaVelocity = MathUtil.applyDeadband(omegaVelocity, .1);

            if ((omegaVelocity == 0 && !hasBeenZero) | (Math.abs(DrivetrainSubsystem.getInstance().getPose().getRotation().minus(lockedHeading).getDegrees()) > 2)) {
                hasBeenZero = true;
                lockedHeading = DrivetrainSubsystem.getInstance().getPose().getRotation();
            }else if (omegaVelocity != 0) {
                hasBeenZero = false;
            }

            if (omegaVelocity == 0 && Math.abs(DrivetrainSubsystem.getInstance().getRate().in(Units.DegreesPerSecond)) < 20) {
                verticalVelocity = xLimiter.calculate(verticalVelocity);
                horizontalVelocity = yLimiter.calculate(horizontalVelocity);

                DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityThetaControlFOC);
                DrivetrainSubsystem.getInstance().setVelocityThetaControlFOC(horizontalVelocity,verticalVelocity, lockedHeading,true);
            }else{
                omegaVelocity = omegaVelocity * DrivetrainConstants.MAX_ANGULAR_VELOCITY;

                verticalVelocity = xLimiter.calculate(verticalVelocity);
                horizontalVelocity = yLimiter.calculate(horizontalVelocity);
                ChassisSpeeds targetSpeeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omegaVelocity);
                targetSpeeds = limitSpeeds(targetSpeeds);

                DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
                DrivetrainSubsystem.getInstance().setVelocityFOC(targetSpeeds);
            }

            alignLastTick = false;

            DrivetrainSubsystem.getInstance().setPositionalControl(false);

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
                }
                alignLastTick = true;
            }

            double xOutput = xAlignController.calculate(currentPose.getX(), targetPose.getX());
            double yOutput = yAlignController.calculate(currentPose.getY(), targetPose.getY());
            double omegaOutput = omegaAlignController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            DrivetrainSubsystem.getInstance().setTargetPoseLog(targetPose, targetPose.getX(),targetPose.getY(), targetPose.getRotation().getRadians(), xOutput,yOutput,omegaOutput,xAlignController.atSetpoint(),yAlignController.atSetpoint(),omegaAlignController.atSetpoint());
            DrivetrainSubsystem.getInstance().setPositionalControl(true);

            ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, omegaOutput);
            speeds = limitSpeeds(speeds);

            DrivetrainSubsystem.getInstance().setVelocityFOC(speeds);
            DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);

        } else {
            alignLastTick = false;
            Pose2d currentPose = DrivetrainSubsystem.getInstance().getPose();
            Pose2d targetPose;
            if (leftSourceB) {
                targetPose = LEFT_SOURCE;
            }else {
                targetPose = RIGHT_SOURCE;
            }

            double xOutput = xAlignController.calculate(currentPose.getX(), targetPose.getX());
            double yOutput = yAlignController.calculate(currentPose.getY(), targetPose.getY());
            double omegaOutput = omegaAlignController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            DrivetrainSubsystem.getInstance().setTargetPoseLog(targetPose, targetPose.getX(),targetPose.getY(), targetPose.getRotation().getRadians(), xOutput,yOutput,omegaOutput,xAlignController.atSetpoint(),yAlignController.atSetpoint(),omegaAlignController.atSetpoint());
            DrivetrainSubsystem.getInstance().setPositionalControl(true);

            ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, omegaOutput);
            speeds = limitSpeeds(speeds);

            DrivetrainSubsystem.getInstance().setVelocityFOC(speeds);
            DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
        }
    }

    public ChassisSpeeds limitSpeeds(ChassisSpeeds speeds) {
        Rotation2d heading = new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double linearVelocity = Math.sqrt((speeds.vxMetersPerSecond * speeds.vxMetersPerSecond) + (speeds.vyMetersPerSecond * speeds.vyMetersPerSecond));
        double rotationalLinearVelocity = speeds.omegaRadiansPerSecond * (DrivetrainConstants.DRIVETRAIN_LENGTH/2);
        double totalLinearVelocity = linearVelocity + rotationalLinearVelocity;

        double rotationalProportion = rotationalLinearVelocity / totalLinearVelocity;
        double linearProportion = linearVelocity / totalLinearVelocity;

        totalLinearVelocity = MathUtil.clamp(totalLinearVelocity, -DrivetrainConstants.MAX_VELOCITY, DrivetrainConstants.MAX_VELOCITY);
//        totalLinearVelocity = velocityLimiter.calculate(totalLinearVelocity);

        double limitedLinearVelocity = totalLinearVelocity * linearProportion;

        speeds.vxMetersPerSecond = limitedLinearVelocity * heading.getCos();
        speeds.vyMetersPerSecond = limitedLinearVelocity * heading.getSin();

        speeds.omegaRadiansPerSecond = (totalLinearVelocity * rotationalProportion) / (DrivetrainConstants.DRIVETRAIN_LENGTH/2); // v=wr

        return speeds;
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
