package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.algae.AlgaePivotStates;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralMotorState;
import frc.robot.subsystems.coral.CoralPistonState;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class CommandFactory {

    public static Command getScoreAlgaeCommand() {
        return new InstantCommand(() -> {});
    }
    public static Command getScoreCoralCommand() {
        return new InstantCommand(() -> {});
    }

    public static Command PositionL4() {
        return new InstantCommand(
                () -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L4);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
                    AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
                }
        );
    }

    public static Command PositionL3() {
        return new InstantCommand(
                () -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L3);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
                    AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
                }
        );
    }

    public static Command PositionL2() {
        return new InstantCommand(
                () -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.L2);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
                    AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
                }
        );
    }

    public static Command PositionTrough() {
        return new InstantCommand(
                () -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Trough);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Reef);
                    AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
                }
        );
    }

    public static Command Source() {
        return new InstantCommand(() -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Source);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Source);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Source);
                    AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
                }
        );
    }

    public static Command Processor() {
        return new InstantCommand(
                () -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Processor);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Store);
                    AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Processor);
                }
        );
    }

    public static Command PositionBarge() {
        return new InstantCommand(
                () -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.Barge);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Store);
                    AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Barge);
                }
        );
    }

    public static Command StoreCoral() {
        return new InstantCommand(
                () -> {
                    ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreCoral);
                    CoralSubsystem.getInstance().setCoralPistonState(CoralPistonState.Hold);
                    CoralSubsystem.getInstance().setCoralMotorState(CoralMotorState.Off);
                    AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
                }
        );
    }

    public static Command StoreAlgae() {
        return new InstantCommand();
    }

    public static Command Score() {
        return new InstantCommand();
    }
}
