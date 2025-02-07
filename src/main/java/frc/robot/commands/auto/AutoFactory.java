package frc.robot.commands.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Superstructure;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.HashMap;
import java.util.Optional;

public class AutoFactory {

    public enum StartOptions {
        Right,
        Center,
        Left,
    }

    public enum ReefOptions {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
        None,
    }

    public enum SourceOptions {
        Left,
        Right
    }

    private final HashMap<String, Trajectory<SwerveSample>> trajectoriesBlue = new HashMap<>();
    private final HashMap<String, Trajectory<SwerveSample>> trajectoriesRed = new HashMap<>();

    private final SendableChooser<StartOptions> startOptions = new SendableChooser<>();
    private final SendableChooser<SourceOptions> sourceOptions = new SendableChooser<>();

    private SendableChooser<ReefOptions> firstScoreOptions = new SendableChooser<>();
    private SendableChooser<ReefOptions> secondScoreOptions = new SendableChooser<>();
    private SendableChooser<ReefOptions> thirdScoreOptions = new SendableChooser<>();

    private Command autoCommand = new InstantCommand(() -> System.out.println("No auto loaded"));

    public AutoFactory() {
        trajectoriesRed.clear();
        trajectoriesBlue.clear();
        
        loadTrajectory("A_LS");
        loadTrajectory("A_RS");
        loadTrajectory("B_LS");
        loadTrajectory("B_RS");

        loadTrajectory("CST_E");
        loadTrajectory("CST_F");
        loadTrajectory("CST_G");
        loadTrajectory("CST_H");
        loadTrajectory("CST_I");
        loadTrajectory("CST_J");
        loadTrajectory("C_RS");
        loadTrajectory("D_RS");
        loadTrajectory("E_RS");
        loadTrajectory("F_RS");
        loadTrajectory("G_LS");
        loadTrajectory("G_RS");
        loadTrajectory("H_LS");
        loadTrajectory("H_RS");
        loadTrajectory("I_LS");
        loadTrajectory("J_LS");
        loadTrajectory("K_LS");
        loadTrajectory("LST_G");
        loadTrajectory("LST_H");
        loadTrajectory("LST_I");
        loadTrajectory("LST_J");
        loadTrajectory("LST_K");
        loadTrajectory("LST_L");
        loadTrajectory("LS_A");
        loadTrajectory("LS_B");
        loadTrajectory("LS_G");
        loadTrajectory("LS_H");
        loadTrajectory("LS_I");
        loadTrajectory("LS_J");
        loadTrajectory("LS_K");
        loadTrajectory("LS_L");
        loadTrajectory("L_LS");
        loadTrajectory("RST_C");
        loadTrajectory("RST_D");
        loadTrajectory("RST_E");
        loadTrajectory("RST_F");
        loadTrajectory("RST_G");
        loadTrajectory("RST_H");
        loadTrajectory("RS_A");
        loadTrajectory("RS_B");
        loadTrajectory("RS_C");
        loadTrajectory("RS_D");
        loadTrajectory("RS_E");
        loadTrajectory("RS_F");
        loadTrajectory("RS_G");
        loadTrajectory("RS_H");

        startOptions.setDefaultOption("Center", StartOptions.Center);
        startOptions.addOption("Right", StartOptions.Right);
        startOptions.addOption("Left", StartOptions.Left);

        sourceOptions.setDefaultOption("Right", SourceOptions.Right);
        sourceOptions.addOption("Left", SourceOptions.Left);

        sourceOptions.onChange(event -> {updateOptions(); generateAuto();});
        startOptions.onChange(event -> {updateOptions(); generateAuto();});

        firstScoreOptions.setDefaultOption("H", ReefOptions.H);
        secondScoreOptions.setDefaultOption("H", ReefOptions.H);
        thirdScoreOptions.setDefaultOption("H", ReefOptions.H);

        SmartDashboard.putData("Starting Location", startOptions);
        SmartDashboard.putData("Source Location", sourceOptions);

        SmartDashboard.putData("First Score", firstScoreOptions);
        SmartDashboard.putData("Second Score", secondScoreOptions);
        SmartDashboard.putData("Third Score", thirdScoreOptions);

        updateOptions();
    }

    private void setLeftSecondAndThirdOptions() {

        secondScoreOptions.addOption("A", ReefOptions.A);
        secondScoreOptions.addOption("B", ReefOptions.B);
        secondScoreOptions.addOption("G", ReefOptions.G);
        secondScoreOptions.addOption("H", ReefOptions.H);
        secondScoreOptions.addOption("I", ReefOptions.I);
        secondScoreOptions.addOption("J", ReefOptions.J);
        secondScoreOptions.addOption("K", ReefOptions.K);
        secondScoreOptions.addOption("L", ReefOptions.L);

        secondScoreOptions.setDefaultOption("None", ReefOptions.None);

        thirdScoreOptions.addOption("A", ReefOptions.A);
        thirdScoreOptions.addOption("B", ReefOptions.B);
        thirdScoreOptions.addOption("G", ReefOptions.G);
        thirdScoreOptions.addOption("H", ReefOptions.H);
        thirdScoreOptions.addOption("I", ReefOptions.I);
        thirdScoreOptions.addOption("J", ReefOptions.J);
        thirdScoreOptions.addOption("K", ReefOptions.K);
        thirdScoreOptions.addOption("L", ReefOptions.L);

        thirdScoreOptions.setDefaultOption("None", ReefOptions.None);
    }

    private void setRightSecondAndThirdOptions() {
        secondScoreOptions.addOption("A", ReefOptions.A);
        secondScoreOptions.addOption("B", ReefOptions.B);
        secondScoreOptions.addOption("C", ReefOptions.C);
        secondScoreOptions.addOption("D", ReefOptions.D);
        secondScoreOptions.addOption("E", ReefOptions.E);
        secondScoreOptions.addOption("F", ReefOptions.F);
        secondScoreOptions.addOption("G", ReefOptions.G);
        secondScoreOptions.addOption("H", ReefOptions.H);

        secondScoreOptions.setDefaultOption("None", ReefOptions.None);

        thirdScoreOptions.addOption("A", ReefOptions.A);
        thirdScoreOptions.addOption("B", ReefOptions.B);
        thirdScoreOptions.addOption("C", ReefOptions.C);
        thirdScoreOptions.addOption("D", ReefOptions.D);
        thirdScoreOptions.addOption("E", ReefOptions.E);
        thirdScoreOptions.addOption("F", ReefOptions.F);
        thirdScoreOptions.addOption("G", ReefOptions.G);
        thirdScoreOptions.addOption("H", ReefOptions.H);

        thirdScoreOptions.setDefaultOption("None", ReefOptions.None);
    }

    private void updateOptions() {
        firstScoreOptions.close();
        secondScoreOptions.close();
        thirdScoreOptions.close();

        SmartDashboard.updateValues();

        StartOptions startSelection = startOptions.getSelected();
        SourceOptions sourceSelection = sourceOptions.getSelected();

        firstScoreOptions = new SendableChooser<>();
        secondScoreOptions = new SendableChooser<>();
        thirdScoreOptions = new SendableChooser<>();

        switch (startSelection) {
            case Left : {
                switch (sourceSelection) {
                    case Left: {
                        /*
                         * First Selection can be
                         * G, H, I, J, K, L
                         * Second and third selection can be
                         * L, K, J, I, H, G, B, A
                         */


                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        firstScoreOptions.addOption("I", ReefOptions.I);
                        firstScoreOptions.addOption("J", ReefOptions.J);
                        firstScoreOptions.addOption("K", ReefOptions.K);
                        firstScoreOptions.addOption("L", ReefOptions.L);

                        setLeftSecondAndThirdOptions();
                        break;

                    }
                    case Right: {
                        /*
                        First Selection can be
                        G, H
                        Second and third selection can be
                        A, B, C, D, E, F, G, H
                         */

                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);

                        setRightSecondAndThirdOptions();
                        break;
                    }
                }
                break;
            }
            case Center : {
                switch (sourceSelection) {
                    case Left: {
                        /*
                         First selection can be
                         G, H, J, I,
                         Second and third selection can be
                          L, K, J, I, H, G, B, A
                         */

                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        firstScoreOptions.addOption("I", ReefOptions.I);
                        firstScoreOptions.addOption("J", ReefOptions.J);
                        setLeftSecondAndThirdOptions();
                        break;
                    }
                    case Right: {
                        /*
                        First selection can be
                        G, H, F, E
                        Second and third selection can be
                        A, B, C, D, E, F, G, H
                         */
                        setRightSecondAndThirdOptions();

                        firstScoreOptions.addOption("E", ReefOptions.E);
                        firstScoreOptions.addOption("F", ReefOptions.F);
                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        break;
                    }
                }
                break;
            }
            case Right : {
                switch (sourceSelection) {
                    case Left: {
                        /*
                        First Selection can be
                        G, H
                        Second and third selection can be

                         L, K, J, I, H, G, B, A
                         */

                        setLeftSecondAndThirdOptions();
                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        break;
                    }
                    case Right: {
                        /*
                        First selection can be
                        H, G, F, E, C, D
                        Second and third selections
                        H, G, F, E, C, D, A, B
                         */

                        firstScoreOptions.addOption("C", ReefOptions.C);
                        firstScoreOptions.addOption("D", ReefOptions.D);
                        firstScoreOptions.addOption("E", ReefOptions.E);
                        firstScoreOptions.addOption("F", ReefOptions.F);
                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);

                        setRightSecondAndThirdOptions();
                        break;
                    }

                }
                break;
            }
        }

        SmartDashboard.putData("First Score", firstScoreOptions);
        SmartDashboard.putData("Second Score", secondScoreOptions);
        SmartDashboard.putData("Third Score", thirdScoreOptions);
        SmartDashboard.updateValues();

        firstScoreOptions.onChange(event -> {generateAuto();});
        secondScoreOptions.onChange(event -> {generateAuto();});
        thirdScoreOptions.onChange(event -> {generateAuto();});
    }

    private void generateAuto() {
        StartOptions startSelection = startOptions.getSelected();
        SourceOptions sourceSelection = sourceOptions.getSelected();

        ReefOptions reefTarget1 = firstScoreOptions.getSelected();
        ReefOptions reefTarget2 = secondScoreOptions.getSelected();
        ReefOptions reefTarget3 = thirdScoreOptions.getSelected();
        String startingLocation = "";
        switch (startSelection) {
            case Left : {
                startingLocation = "LST";
                break;
            }
            case Right : {
                startingLocation = "RST";
                break;
            }
            case Center : {
                startingLocation = "CST";
                break;
            }
        }
        String sourceLocation = "";
        switch (sourceSelection) {
            case Left: {
                sourceLocation = "LS";
                break;
            }
            case Right: {
                sourceLocation = "RS";
                break;
            }
        }
        if (reefTarget1 == null || reefTarget2 == null || reefTarget3 == null) {
            System.out.println("Error Generating Auto");
            autoCommand = new InstantCommand(() -> System.out.println("Problem with auto"));
            return;
        }

        String startPath = startingLocation + "_" + reefTarget1.toString();
        String toSource1 = reefTarget1.toString() + "_" + sourceLocation;
        String sourceTo2 = sourceLocation + "_" + reefTarget2.toString();
        String toSource2 = reefTarget2.toString() + "_" + sourceLocation;
        String sourceTo3 = sourceLocation + "_" + reefTarget3.toString();
        Trajectory<SwerveSample> part1;
        Trajectory<SwerveSample> part2;
        Trajectory<SwerveSample> part3;
        Trajectory<SwerveSample> part4;
        Trajectory<SwerveSample> part5;
        if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
            part1 = trajectoriesBlue.get(startPath);
            part2 = trajectoriesBlue.get(toSource1);
            part3 = trajectoriesBlue.get(sourceTo2);
            part4 = trajectoriesBlue.get(toSource2);
            part5 = trajectoriesBlue.get(sourceTo3);
        }else{
            part1 = trajectoriesRed.get(startPath);
            part2 = trajectoriesRed.get(toSource1);
            part3 = trajectoriesRed.get(sourceTo2);
            part4 = trajectoriesRed.get(toSource2);
            part5 = trajectoriesRed.get(sourceTo3);
        }

        DrivetrainSubsystem.getInstance().resetPose(part1.getInitialPose(false).get());
        autoCommand = new SequentialCommandGroup(
                generateFollowTrajectoryCommand(part1),
                new InstantCommand(() -> System.out.println("Score 1")),
                new WaitCommand(1),
                generateFollowTrajectoryCommand(part2),
                new WaitCommand(1),
                new InstantCommand(() -> System.out.println("Pickup from source")),
                generateFollowTrajectoryCommand(part3),
                new InstantCommand(() -> System.out.println("Score 2")),
                new WaitCommand(1),
                generateFollowTrajectoryCommand(part4),
                new InstantCommand(() -> System.out.println("Pickup from source")),
                new WaitCommand(1),
                generateFollowTrajectoryCommand(part5),
                new InstantCommand(() -> System.out.println("Score 3"))
        );
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    public void loadTrajectory(String trajectoryName) {
        Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(trajectoryName);
        if (trajectory.isPresent()) {
            trajectoriesBlue.put(trajectoryName, trajectory.get());
            trajectoriesRed.put(trajectoryName, trajectory.get().flipped());
            System.out.println("Trajectory loaded: " + trajectoryName);
        }else{
            System.out.println("Trajectory not found: " + trajectoryName);
        }
    }

    private Command generateFollowTrajectoryCommand(Trajectory<SwerveSample> trajectory) {

        return new SequentialCommandGroup(
                new FollowTrajectory(trajectory)
//                new GoToPose(trajectory.getFinalPose(false).)
        );
    }
}
