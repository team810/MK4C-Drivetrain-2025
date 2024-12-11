package frc.robot.commands.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class Interlope extends Command {

    private Pose2d currentPose;
    private final SwerveSample target;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public Interlope(SwerveSample target) {
        this.target = target;
        xController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);
        thetaController = new PIDController(0, 0, 0);
    }

    @Override
    public void initialize() {
        currentPose = DrivetrainSubsystem.getInstance().getPose();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return
            MathUtil.isNear(target.getPose().getY(),currentPose.getX(), DrivetrainConstants.INTERLOPE_TOLERANCE.in(Meters)) &&
            MathUtil.isNear(target.getPose().getX(),currentPose.getY(), DrivetrainConstants.INTERLOPE_TOLERANCE.in(Meters)) &&
            MathUtil.isNear(target.getPose().getRotation().getRadians(), currentPose.getRotation().getRadians(), DrivetrainConstants.INTERLOPE_ANGlE_TOLERANCE.in(Radians))
            ;
    }

}
