package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    public static final double PERIOD = .020;

    private final AutoFactory autoFactory;

    public Robot()
    {
        super(PERIOD);
        Logger.recordMetadata("ProjectName", "Reefscape"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
        } else {

//            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
//            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
//            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        }

        Logger.start();

        DriverStation.silenceJoystickConnectionWarning(true);
        CommandScheduler.getInstance().setPeriod(.015);
        Superstructure.getInstance().initialize();
        CommandScheduler.getInstance().unregisterSubsystem(DrivetrainSubsystem.getInstance(), VisionSubsystem.getInstance());
        autoFactory = new AutoFactory();
    }
    @Override
    public void robotInit() {
        Superstructure.getInstance().configureActions();
    }


    public DrivetrainSubsystem DrivetrainSubsystem() {
        return DrivetrainSubsystem.getInstance();
    }


    public VisionSubsystem VisionSubsystem() {
        return VisionSubsystem.getInstance();
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
        CommandScheduler.getInstance().schedule(autoFactory.getAutoCommand());
    }
    
    
    @Override
    public void autonomousPeriodic() {

    }

    ManualDriveCommand manualDriveCommand;

    @Override
    public void teleopInit() {
        manualDriveCommand = new ManualDriveCommand();
        CommandScheduler.getInstance().schedule(manualDriveCommand);
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
