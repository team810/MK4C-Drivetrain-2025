package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoAlignGP extends Command {
    private final PIDController xController;
    private final PIDController yController;

    public AutoAlignGP() {
        xController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);

        xController.setSetpoint(2);
        yController.setSetpoint(2);
    }

    @Override
    public void execute() {
        if (VisionSubsystem.getInstance().isTargetDetected()) {
            Transform2d transform2d = VisionSubsystem.getInstance().getTargetTransform();

            double xOutput = -xController.calculate(transform2d.getX(), -.67);
            double yOutput = yController.calculate(transform2d.getY(), 0);

            xOutput = MathUtil.clamp(xOutput, -3,3);
            yOutput = MathUtil.clamp(yOutput, -3,3);

            DrivetrainSubsystem.getInstance().setVelocityRR(new ChassisSpeeds(yOutput, xOutput,0));
            DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityRR);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint();
    }
}
