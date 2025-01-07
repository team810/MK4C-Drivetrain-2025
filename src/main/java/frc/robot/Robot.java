
package frc.robot;


import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.auto.AutoAlignGP;
import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.commands.auto.Interlope;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import jdk.jshell.execution.Util;
import java.util.Optional;

@Logged
public class Robot extends TimedRobot {
    public static final double PERIOD = .020;

    public Robot()
    {
        super(PERIOD);
        Epilogue.bind(this);
//        Logger.recordMetadata("ProjectName", "Swerve Drivetrain");
        DriverStation.silenceJoystickConnectionWarning(true);
//
//        if (isReal()) {
//            Logger.addDataReceiver(new NT4Publisher());
//            Logger.addDataReceiver(new WPILOGWriter());
//        } else {
//            Logger.addDataReceiver(new NT4Publisher());
//        }
//        Logger.start();
        CommandScheduler.getInstance().setPeriod(.015);

        Superstructure.getInstance().initialize();
        CommandScheduler.getInstance().unregisterSubsystem(DrivetrainSubsystem.getInstance(), VisionSubsystem.getInstance());
    }
    @Override
    public void robotInit() {
        Superstructure.getInstance().configureActions();
    }

    @Logged
    public DrivetrainSubsystem DrivetrainSubsystem() {
        return DrivetrainSubsystem.getInstance();
    }
    @Logged
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

//        Optional<Trajectory<SwerveSample>> path = Choreo.loadTrajectory("test");
//        if (path.isEmpty()) {
//            System.out.println("not found");
//
//        }
//        Trajectory<SwerveSample> part1 = path.get().getSplit(0).get();
//        Trajectory<SwerveSample> part2 = path.get().getSplit(2).get();
//        SwerveSample part2Start = part2.getInitialSample(false).get();
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetPose(path.get().getInitialPose(false).get())),
//                        new FollowTrajectory(part1),
//                        new AutoAlignGP(),
//                        new Interlope(part2Start.getPose(), part2Start.vx, part2Start.vy, part2Start.omega),
//                        new FollowTrajectory(part2)
//                )
//        );
    }
    
    
    @Override
    public void autonomousPeriodic() {

    }
    
    @Logged (name = "Drive Command")
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
