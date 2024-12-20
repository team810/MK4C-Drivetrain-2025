
package frc.robot;


import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAlert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.Optional;

public class Robot extends LoggedRobot {
    public static final double PERIOD = .020;

    public Robot()
    {
        super(PERIOD);
        
        Logger.recordMetadata("ProjectName", "Swerve Drivetrain");
        DriverStation.silenceJoystickConnectionWarning(true);
        
        if (isReal()) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.start();
        CommandScheduler.getInstance().setPeriod(.015);

        Superstructure.getInstance().initialize();
        CommandScheduler.getInstance().unregisterSubsystem(DrivetrainSubsystem.getInstance(), VisionSubsystem.getInstance());
    }
    @Override
    public void robotInit() {
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
        VisionSubsystem.getInstance().readPeriodic();
    }

    public void writePeriodic() {

        DrivetrainSubsystem.getInstance().writePeriodic();

        VisionSubsystem.getInstance().writePeriodic();
    }
    
    @Override
    public void autonomousInit() {

        Optional<Trajectory<SwerveSample>> path = Choreo.loadTrajectory("test");
        if (path.isEmpty()) {
            System.out.println("not found");
        }
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetPose(path.get().getInitialPose(false).get())),
                        new FollowTrajectory(path.get())
                )
        );
    }
    
    
    @Override
    public void autonomousPeriodic() {

    }
    
    
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(new ManualDriveCommand());
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void disabledInit() {
    }
    
    
    @Override
    public void disabledPeriodic() {
        Superstructure.getInstance().disabledPeriodic();
    }

    @Override
    public void simulationPeriodic() {
        DrivetrainSubsystem.getInstance().simulationPeriodic();
        VisionSubsystem.getInstance().simulationPeriodic();
    }
}
