package frc.robot;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.ScoreAlgae;
import frc.robot.subsystems.algae.AlgaePivotStates;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;

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
        pneumaticsControlModule.disableCompressor();

        DrivetrainSubsystem.getInstance().resetPose(new Pose2d(0, 0, new Rotation2d(0)));

        alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    }

    public void configureActions() {
//        new Trigger(IO.getButtonValue(Controls.resetGyro)).onTrue(new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyro()));
        new Trigger(IO.getButtonValue(Controls.resetGyro)).whileTrue(new IntakeAlgae());
        new Trigger(IO.getButtonValue(Controls.PositionProcessor)).onTrue(new InstantCommand(() -> {AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Processor);}));
        new Trigger(IO.getButtonValue(Controls.Score)).whileTrue(new ScoreAlgae());
        new Trigger(IO.getButtonValue(Controls.PositionBarge)).onTrue(new InstantCommand(() -> {AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Barge);}));

//        new Trigger(IO.getButtonValue(Controls.PositionBarge)).onTrue(CommandFactory.PositionBarge());
//        new Trigger(IO.getButtonValue(Controls.PositionProcessor)).onTrue(CommandFactory.Processor());
//        new Trigger(IO.getButtonValue(Controls.Source)).onTrue(CommandFactory.Source());
//        new Trigger(IO.getButtonValue(Controls.PositionStore)).onTrue(CommandFactory.StoreCoral());
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

        Pose3d secondStage = new Pose3d(0,0,ElevatorSubsystem.getInstance().getCurrentHeight().in(Meters) / 3, new Rotation3d());
        Pose3d insideStage = new Pose3d(0,0, ElevatorSubsystem.getInstance().getCurrentHeight().in(Meters) / 2, new Rotation3d());

        Pose3d carriage = new Pose3d(0, .1, (ElevatorSubsystem.getInstance().getCurrentHeight().in(Meters)) + .18,new Rotation3d());
        Pose3d algae = new Pose3d(0, .2,carriage.getZ() + .17,new Rotation3d(Math.toRadians(-63) + AlgaeSubsystem.getInstance().currentPivotAngle(),0,0));
        Pose3d coral = new Pose3d(.15,.23,carriage.getZ() + .07,CoralSubsystem.getInstance().getAngle());

        Pose3d coralGP = new Pose3d(0,0,0,new Rotation3d());
        if (CoralSubsystem.getInstance().hasCoral()) {
            coralGP = coral;
        }
        Pose3d algaeGP = new Pose3d(0,0,0,new Rotation3d());
        if (AlgaeSubsystem.getInstance().hasAlgae()) {
            Pose3d pose = new Pose3d(DrivetrainSubsystem.getInstance().getPose());
            pose = pose.plus(new Transform3d(0,.2, carriage.getZ() + .17, new Rotation3d()));
            double angle = AlgaeSubsystem.getInstance().currentPivotAngle();
            double zOffset = .33 * Math.sin(angle);
            double yOffset = .33 * Math.cos(angle);
            pose = pose.plus(new Transform3d(0,yOffset,zOffset, new Rotation3d()));
            algaeGP = pose;
        }
        Logger.recordOutput("Mechanism", secondStage, insideStage, carriage, algae, coral);
        Logger.recordOutput("CoralGP", coralGP);
        Logger.recordOutput("AlgaeGP", algaeGP);

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
