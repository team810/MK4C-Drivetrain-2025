package frc.robot.commands.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.ArrayList;
import java.util.Optional;

public class AutoFactory {

    public AutoFactory() {

    }

    private void loadAllPaths() {

    }

    private static class Path {
        private final String name;
        private final double numOfSegments; // This is the total number of segments in the path
        private final ArrayList<Double> ghostSegments; // These are the segments where the robot drives to the game piece, these segments get taken over by the vision
        private ArrayList<Trajectory<SwerveSample>> trajectories;

        private Command autoCommand;
        private Pose2d startPose;

        public Path(String name, double numberOfSegments, ArrayList<Double> ghostSegments) {
            this.name = name;
            this.numOfSegments = numberOfSegments;
            this.ghostSegments = ghostSegments;

            Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(name);
            if (trajectory.isPresent()) {
                for (int i = 0; i < numberOfSegments; i++) {
                    Optional<Trajectory<SwerveSample>> split = trajectory.get().getSplit(i);
                    if (split.isPresent()) {
                        trajectories.add(split.get());
                    }else{
                        System.out.println("There is no slip for " + i);
                        break;
                    }
                }
            }else{
                System.out.println("Failed to load path: " + name);
                return;
            }

        }
    }


    private Command getScoreCommand() { // This is just a placeholder function
        return new SequentialCommandGroup(
                new InstantCommand(()->{System.out.println("Start Score");}),
                new WaitCommand(1),
                new InstantCommand(() -> {System.out.println("Finish Score");})
        );
    }

    private Command getIntakeCommand() { // This is just a placeholder function
        return new SequentialCommandGroup(
                new InstantCommand(()->{System.out.println("Intake Start");}),
                new WaitCommand(1),
                new InstantCommand(() -> {System.out.println("Intake Finish");})
        );
    }
}
