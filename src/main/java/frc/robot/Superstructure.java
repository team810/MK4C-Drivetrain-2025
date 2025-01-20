package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Superstructure {
    private static Superstructure instance;
    private DriverStation.Alliance alliance;
    private final AutoFactory autoFactory;


    public Superstructure() {
        autoFactory = new AutoFactory();
        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    }

    public void initialize() {
        IO.Initialize(IO.PrimaryDriverProfiles.Leo,IO.SecondaryDriverProfiles.KnollController);
        DrivetrainSubsystem.getInstance();
        DrivetrainSubsystem.getInstance().resetPose(new Pose2d(0, 0, new Rotation2d(0)));
        VisionSubsystem.getInstance();
    }

    public void configureActions() {
        new Trigger(IO.getButtonValue(Controls.resetGyro)).onTrue(new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyro()));

    }

    public void disabledPeriodic() {
        // updating alliance selection
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            alliance = DriverStation.Alliance.Blue;
        }else{
            alliance = DriverStation.Alliance.Red;
        }
        if (!DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(alliance)) {
            DrivetrainSubsystem.getInstance().switchAlliances();
        }


    }

    public DriverStation.Alliance getAlliance() {
        return alliance;
    }

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }
}
