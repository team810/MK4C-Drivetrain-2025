package frc.robot;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
    private static Superstructure instance;
    private DriverStation.Alliance alliance;

    private GamePiece currentPiece;

    private final PneumaticsControlModule pneumaticsControlModule;

    public Superstructure() {
        ChoreoAllianceFlipUtil.setYear(2025);
        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        IO.Initialize(IO.PrimaryDriverProfiles.Leo,IO.SecondaryDriverProfiles.KnollController);
        DrivetrainSubsystem.getInstance();
        ElevatorSubsystem.getInstance();
        CoralSubsystem.getInstance();
        AlgaeSubsystem.getInstance();

        pneumaticsControlModule = new PneumaticsControlModule(); // Idk which one to use
//        pneumaticsControlModule.enableCompressorAnalog(0, 120);
//        pneumaticsControlModule.enableCompressorDigital();

        DrivetrainSubsystem.getInstance().resetPose(new Pose2d(0, 0, new Rotation2d(0)));
        ClimberSubsystem.getInstance().changeAlliance(alliance);

        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    }

    public void configureActions() {
        new Trigger(IO.getButtonValue(Controls.resetGyro)).onTrue(new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyro()));
    }

    public void periodic() {
        if (CoralSubsystem.getInstance().hasCoral() && AlgaeSubsystem.getInstance().hasAlgae()) {
            currentPiece = GamePiece.Both;
        } else if (CoralSubsystem.getInstance().hasCoral() && !AlgaeSubsystem.getInstance().hasAlgae()) {
            currentPiece = GamePiece.Coral;
        } else if (!CoralSubsystem.getInstance().hasCoral() && AlgaeSubsystem.getInstance().hasAlgae()) {
            currentPiece = GamePiece.Algae;
        }else{
            currentPiece = GamePiece.None;
        }

        Logger.recordOutput("SuperStructure/CurrentPiece", currentPiece);
        Logger.recordOutput("SuperStructure/Alliance", alliance);
        Logger.recordOutput("SuperStructure/Pressure", pneumaticsControlModule.getPressure(0));
    }

    public double getCurrentPressure() {
        return pneumaticsControlModule.getPressure(0);
    }

    public GamePiece getCurrentPiece() {
        return currentPiece;
    }

    // Alliance Stuff
    public void disabledPeriodic() {
        // updating alliance selection
        DriverStation.Alliance currentAlliance = alliance;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            alliance = DriverStation.Alliance.Blue;
        }else{
            alliance = DriverStation.Alliance.Red;
        }
        if (!DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(alliance)) {
            DrivetrainSubsystem.getInstance().switchAlliances();
        }
        if (currentAlliance != alliance) {
            // Alliance was changed
            ClimberSubsystem.getInstance().changeAlliance(alliance);
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
