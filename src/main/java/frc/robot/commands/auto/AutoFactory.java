package frc.robot.commands.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    public AutoFactory() {
        startOptions.setDefaultOption("Center", StartOptions.Center);
        startOptions.addOption("Right", StartOptions.Right);
        startOptions.addOption("Center", StartOptions.Center);
        startOptions.addOption("Left", StartOptions.Left);

        sourceOptions.setDefaultOption("Right", SourceOptions.Left);
        sourceOptions.addOption("Left", SourceOptions.Left);
        sourceOptions.addOption("Right", SourceOptions.Right);

        sourceOptions.onChange(event -> {updateOptions();});
        startOptions.onChange(event -> {updateOptions();});

        firstScoreOptions.setDefaultOption("None", ReefOptions.None);
        secondScoreOptions.setDefaultOption("None", ReefOptions.None);
        thirdScoreOptions.setDefaultOption("None", ReefOptions.None);

        SmartDashboard.putData("Starting Location", startOptions);
        SmartDashboard.putData("Source Location", sourceOptions);

        SmartDashboard.putData("First Score", firstScoreOptions);
        SmartDashboard.putData("Second Score", secondScoreOptions);
        SmartDashboard.putData("Third Score", thirdScoreOptions);

        updateOptions();

        System.out.println("Trajectory Loading Started");
        double startTime = Timer.getFPGATimestamp();
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

        System.out.println("Trajectory loading finished : " + (Timer.getFPGATimestamp() - startTime));
    }

    private void setLeftSecondAndThirdOptions() {
//        secondScoreOptions.close();
//        thirdScoreOptions.close();
        secondScoreOptions = new SendableChooser<>();
        thirdScoreOptions = new SendableChooser<>();

        secondScoreOptions.addOption("A", ReefOptions.A);
        secondScoreOptions.addOption("B", ReefOptions.B);
        secondScoreOptions.addOption("G", ReefOptions.G);
        secondScoreOptions.addOption("H", ReefOptions.H);
        secondScoreOptions.addOption("I", ReefOptions.I);
        secondScoreOptions.addOption("J", ReefOptions.J);
        secondScoreOptions.addOption("K", ReefOptions.K);
        secondScoreOptions.addOption("L", ReefOptions.L);

        thirdScoreOptions.addOption("A", ReefOptions.A);
        thirdScoreOptions.addOption("B", ReefOptions.B);
        thirdScoreOptions.addOption("G", ReefOptions.G);
        thirdScoreOptions.addOption("H", ReefOptions.H);
        thirdScoreOptions.addOption("I", ReefOptions.I);
        thirdScoreOptions.addOption("J", ReefOptions.J);
        thirdScoreOptions.addOption("K", ReefOptions.K);
        thirdScoreOptions.addOption("L", ReefOptions.L);
    }

    private void setRightSecondAndThirdOptions() {

        secondScoreOptions = new SendableChooser<>();
        thirdScoreOptions = new SendableChooser<>();

        secondScoreOptions.addOption("A", ReefOptions.A);
        secondScoreOptions.addOption("B", ReefOptions.B);
        secondScoreOptions.addOption("C", ReefOptions.C);
        secondScoreOptions.addOption("D", ReefOptions.D);
        secondScoreOptions.addOption("E", ReefOptions.E);
        secondScoreOptions.addOption("F", ReefOptions.F);
        secondScoreOptions.addOption("G", ReefOptions.G);
        secondScoreOptions.addOption("H", ReefOptions.H);

        thirdScoreOptions.addOption("A", ReefOptions.A);
        thirdScoreOptions.addOption("B", ReefOptions.B);
        thirdScoreOptions.addOption("C", ReefOptions.C);
        thirdScoreOptions.addOption("D", ReefOptions.D);
        thirdScoreOptions.addOption("E", ReefOptions.E);
        thirdScoreOptions.addOption("F", ReefOptions.F);
        thirdScoreOptions.addOption("G", ReefOptions.G);
        thirdScoreOptions.addOption("H", ReefOptions.H);
    }

    private void updateOptions() {
        System.out.println("updateOptions");
        StartOptions startSelection = startOptions.getSelected();
        SourceOptions sourceSelection = sourceOptions.getSelected();

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
                        firstScoreOptions = new SendableChooser<>();

                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        firstScoreOptions.addOption("I", ReefOptions.I);
                        firstScoreOptions.addOption("J", ReefOptions.J);
                        firstScoreOptions.addOption("K", ReefOptions.K);
                        firstScoreOptions.addOption("L", ReefOptions.L);

                        setLeftSecondAndThirdOptions();

                    }
                    case Right: {
                        /*
                        First Selection can be
                        G, H
                        Second and third selection can be
                        A, B, C, D, E, F, G, H
                         */
                        firstScoreOptions = new SendableChooser<>();

                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);

                        setRightSecondAndThirdOptions();
                    }
                }
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
                        firstScoreOptions = new SendableChooser<>();

                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                        firstScoreOptions.addOption("I", ReefOptions.I);
                        firstScoreOptions.addOption("J", ReefOptions.J);
                        setLeftSecondAndThirdOptions();
                    }
                    case Right: {
                        /*
                        First selection can be
                        G, H, F, E
                        Second and third selection can be
                        A, B, C, D, E, F, G, H
                         */
                        setRightSecondAndThirdOptions();
                        firstScoreOptions = new SendableChooser<>();

                        firstScoreOptions.addOption("E", ReefOptions.E);
                        firstScoreOptions.addOption("F", ReefOptions.F);
                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);
                    }
                }
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
                        firstScoreOptions = new SendableChooser<>();

                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);

                        setLeftSecondAndThirdOptions();

                    }
                    case Right: {
                        /*
                        First selection can be
                        H, G, F, E, C, D
                        Second and third selections
                        H, G, F, E, C, D, A, B
                         */
                        firstScoreOptions = new SendableChooser<>();

                        firstScoreOptions.addOption("C", ReefOptions.C);
                        firstScoreOptions.addOption("D", ReefOptions.D);
                        firstScoreOptions.addOption("E", ReefOptions.E);
                        firstScoreOptions.addOption("F", ReefOptions.F);
                        firstScoreOptions.addOption("G", ReefOptions.G);
                        firstScoreOptions.addOption("H", ReefOptions.H);

                        setRightSecondAndThirdOptions();
                    }
                }
            }
        }
        SmartDashboard.clearPersistent("First Score");
        SmartDashboard.putData("First Score", firstScoreOptions);
        SmartDashboard.putData("Second Score", secondScoreOptions);
        SmartDashboard.putData("Third Score", thirdScoreOptions);
        SmartDashboard.updateValues();

    }

    private void generateAuto() {

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
}
