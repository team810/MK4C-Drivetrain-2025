package frc.robot.subsystems.vision;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;
import frc.robot.lib.LimelightHelpers;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class VisionSubsystem extends AdvancedSubsystem {
    private static VisionSubsystem INSTANCE;

    private final String LL2 = "limelight-gp";

    private double targetX;
    private double targetY;
    private boolean targetDetected;

    private final ArrayList<Pose2d> gpPoseList = new ArrayList<>();

    private Transform2d targetTransform = new Transform2d();
    private Pose2d targetPose = new Pose2d();


    public VisionSubsystem() {
        gpPoseList.add(new Pose2d(2.8914051055908203  , 7.087372779846191  , new Rotation2d()));
        gpPoseList.add(new Pose2d(2.8914051055908203 , 5.567087650299072 , new Rotation2d()));
        gpPoseList.add(new Pose2d(2.8914051055908203 ,4.066293716430664 , new Rotation2d()));
    }

    @Override
    public void readPeriodic() {
        if (Robot.isReal()) {
            targetDetected = LimelightHelpers.getTV(LL2);
            if (targetDetected) {
                targetX = LimelightHelpers.getTX(LL2);
                targetY = LimelightHelpers.getTY(LL2);

                double x = (Units.inchesToMeters(20) * Math.tan(Math.toRadians(targetX)));
                double y = (Units.inchesToMeters(20) * Math.tan(Math.toRadians(-targetY) + Math.toRadians(-29.8))) - Units.inchesToMeters(15);
                targetTransform = new Transform2d(y, x, new Rotation2d());
                targetPose = DrivetrainSubsystem.getInstance().getPose().transformBy(targetTransform);
            }
        }else{
            targetDetected = false;
            targetX = 0;
            targetY = 0;
        }
        Logger.recordOutput("Vision/TargetTransform", targetTransform);
        Logger.recordOutput("Vision/GPLocation", new Pose3d(targetPose));
    }

    @Override
    public void writePeriodic() {
        // Work on logging when we can figure out the proportional value to use pixels/meters
    }

    @Override
    public void simulatePeriodic() {
         targetDetected = true;

        targetX = 0;
        targetY = 0;

        targetPose = DrivetrainSubsystem.getInstance().getPose().nearest(gpPoseList);
        targetTransform = new Transform2d(DrivetrainSubsystem.getInstance().getPose(), targetPose);

    }
    public Transform2d getTargetTransform() {
        return targetTransform;
    }
    public Pose2d getTargetPose() {
        return targetPose;
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
