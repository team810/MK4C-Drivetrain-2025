
package frc.robot;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TelopDriveCommand;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    public static final double PERIOD = .025;

    private final ShuffleboardTab robotTab;

    private final ShuffleboardContainer resetPositionLayout;
    public enum OriginOptions {
        backRightCorner,
        backLeftCorner,
        frontLeftCorner,
        frontRightCorner,
        center
    }
    private SendableChooser<OriginOptions> originChooser;
    private final GenericEntry xPositionEntry;
    private final GenericEntry yPositionEntry;
    private final GenericEntry thetaEntry;
    private final Command resetPoseCommand;

    public Robot()
    {
        super(PERIOD);
        
        Logger.recordMetadata("ProjectName", "Swerve Drivetrain");
        DriverStation.silenceJoystickConnectionWarning(true);
        
        if (isReal()) {
            Logger.addDataReceiver(new NT4Publisher());
//            Logger.addDataReceiver(new WPILOGWriter());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.start();
        CommandScheduler.getInstance().setPeriod(.015);


        robotTab = Shuffleboard.getTab("Robot");
        resetPositionLayout = robotTab.getLayout("Reset Position", BuiltInLayouts.kList);

        xPositionEntry = resetPositionLayout.add("X Position", 0).getEntry();
        yPositionEntry = resetPositionLayout.add("Y Position", 0).getEntry();
        thetaEntry = resetPositionLayout.add("Theta", 0).getEntry();

        originChooser = new SendableChooser<OriginOptions>();
        resetPositionLayout.add(originChooser);
        originChooser.setDefaultOption("Center", OriginOptions.center);
        originChooser.addOption("BackRightCorner", OriginOptions.backRightCorner);
        originChooser.addOption("BackLeftCorner", OriginOptions.backLeftCorner);
        originChooser.addOption("FrontLeftCorner", OriginOptions.frontLeftCorner);
        originChooser.addOption("FrontRightCorner", OriginOptions.frontRightCorner);

        resetPoseCommand = new InstantCommand(()-> {
            double xPosition = xPositionEntry.getDouble(0);
            double yPosition = yPositionEntry.getDouble(0);
            Rotation2d theta = Rotation2d.fromDegrees(thetaEntry.getDouble(0));
            switch (originChooser.getSelected())
            {
                case backRightCorner -> {
                    xPosition = xPosition + (DrivetrainConstants.DRIVETRAIN_LENGTH/2);
                    yPosition = yPosition + (DrivetrainConstants.DRIVETRAIN_LENGTH/2);
                }
                case backLeftCorner -> {
                    xPosition = xPosition + (DrivetrainConstants.DRIVETRAIN_LENGTH/2);
                    yPosition = yPosition - (DrivetrainConstants.DRIVETRAIN_LENGTH/2);
                }
                case frontLeftCorner -> {
                    xPosition = xPosition - (DrivetrainConstants.DRIVETRAIN_LENGTH/2);
                    yPosition = yPosition - (DrivetrainConstants.DRIVETRAIN_LENGTH/2);
                }
                case frontRightCorner -> {
                    xPosition = xPosition - (DrivetrainConstants.DRIVETRAIN_LENGTH/2);
                    yPosition = yPosition + (DrivetrainConstants.DRIVETRAIN_LENGTH/2);
                }
                case center -> {
                    xPosition = xPosition; // Do not change
                    yPosition = yPosition; // Do not change
                }
            }

            DrivetrainSubsystem.getInstance().resetPose(
                new Pose2d(
                    xPosition,
                    yPosition,
                    theta
                )
            );
        });

        resetPositionLayout.add("Reset Pose Command", resetPoseCommand);
    }
    @Override
    public void robotInit() {
        Superstructure.getInstance().initialize();
        Superstructure.getInstance().configureActions();
    }

    @Override
    public void robotPeriodic() {
        readPeriodic();
        CommandScheduler.getInstance().run();
        writePeriodic();
    }

    public void readPeriodic() {
        DrivetrainSubsystem.getInstance().readPeriodic();
    }

    public void writePeriodic() {
        DrivetrainSubsystem.getInstance().writePeriodic();
    }
    
    @Override
    public void autonomousInit() {}
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(new TelopDriveCommand());
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void disabledInit() {
    }
    
    
    @Override
    public void disabledPeriodic() {
        Logger.recordOutput("Reset Pose", new Pose2d(xPositionEntry.getDouble(0),yPositionEntry.getDouble(0),Rotation2d.fromRadians(thetaEntry.getDouble(0))));
        Superstructure.getInstance().disabledPeriodic();
    }

    @Override
    public void simulationPeriodic() {
        DrivetrainSubsystem.getInstance().simulationPeriodic();
    }
}
