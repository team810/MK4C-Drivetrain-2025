package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class Superstructure {
    private static Superstructure instance;

    private DriverStation.Alliance alliance;


    public Superstructure() {
        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    }

    public void initialize() {
        IO.Initialize(IO.PrimaryDriverProfiles.Leo,IO.SecondaryDriverProfiles.KnollController);
        DrivetrainSubsystem.getInstance();
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
