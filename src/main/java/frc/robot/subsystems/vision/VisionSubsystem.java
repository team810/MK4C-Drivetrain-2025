package frc.robot.subsystems.vision;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;
import frc.robot.lib.LimelightHelpers;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends AdvancedSubsystem {
    private static VisionSubsystem INSTANCE;

    private final String LL2 = "limelight-gp";

    private double targetX;
    private double targetY;
    private boolean targetDetected;

    private final LinearFilter xFilter;
    private final LinearFilter yFilter;


    public VisionSubsystem() {
        xFilter = LinearFilter.movingAverage(1);
        yFilter = LinearFilter.movingAverage(1);
    }

    @Override
    public void readPeriodic() {
        if (Robot.isReal()) {
            targetDetected = LimelightHelpers.getTV(LL2);
            if (targetDetected) {
                targetX = LimelightHelpers.getTX(LL2);
                targetY = LimelightHelpers.getTY(LL2);

                double x = Units.inchesToMeters(20) * Math.tan(Math.toRadians(targetX));
                double y = Units.inchesToMeters(20) * Math.tan(Math.toRadians(targetY) + Math.toRadians(-29.8));

                Logger.recordOutput("GPLocation", DrivetrainSubsystem.getInstance().getPose().transformBy(new Transform2d(y, x, new Rotation2d())));

//                targetX = xFilter.calculate(targetX);
//                targetY = yFilter.calculate(targetY);
            }else{
                xFilter.reset();
                yFilter.reset();
            }
        }else{
            targetDetected = false;
            targetX = 0;
            targetY = 0;
        }
    }

    @Override
    public void writePeriodic() {
        // Work on logging when we can figure out the proportional value to use pixels/meters
    }

    @Override
    public void simulatePeriodic() {
         // Need to find the proportional value to make sim accurate.
    }

    public double getTargetX() {
        return targetX;
    }
    public double getTargetY() {
        return targetY;
    }
    public boolean isTargetDetected() {
        return targetDetected;
    }

    public static VisionSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VisionSubsystem();
        }
        return INSTANCE;
    }
}
