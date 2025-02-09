package frc.robot;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.algae.AlgaeConstants;
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
//        new Trigger(IO.getButtonValue(Controls.resetGyro)).onTrue(new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyro()));
        new Trigger(IO.getButtonValue(Controls.PositionL4)).onTrue(CommandFactory.PositionL4());
        new Trigger(IO.getButtonValue(Controls.PositionStore)).onTrue(CommandFactory.StoreCoral());
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

        Pose3d elevator = new Pose3d(0, .1, edu.wpi.first.math.util.Units.inchesToMeters(ElevatorSubsystem.getInstance().getCurrentHeight()) + 0.18,new Rotation3d());
//        Pose3d elevator = new Pose3d(0, 0,0,new Rotation3d());

        Pose3d algae = new Pose3d(0, .2,elevator.getZ() + .17,new Rotation3d(Math.toRadians(-63) + AlgaeSubsystem.getInstance().currentPivotAngle(),0,0));
        Pose3d coral = new Pose3d(.15,.23,elevator.getZ() + .07,CoralSubsystem.getInstance().getAngle());
//        Pose3d coral = new Pose3d(0,0,0,new Rotation3d(0, 0,0));

        Pose3d coralGP = new Pose3d(0,0,0,new Rotation3d());
        if (CoralSubsystem.getInstance().hasCoral()) {
            coralGP = coral;
        }
        Pose3d algaeGP = new Pose3d(0,0.3048,0,new Rotation3d());
        if (AlgaeSubsystem.getInstance().hasAlgae()) {
            algaeGP = algae;
        }
        Logger.recordOutput("Mechanism", elevator, algae, coral);
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
