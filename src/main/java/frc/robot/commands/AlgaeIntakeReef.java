package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeDriveStates;
import frc.robot.subsystems.algae.AlgaePivotStates;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class AlgaeIntakeReef extends Command {
    public enum TargetHeight {
        High,
        Low
    }

    private final TargetHeight targetHeight;
    public AlgaeIntakeReef(TargetHeight targetHeight) {
        this.targetHeight = targetHeight;
    }

    @Override
    public void initialize() {
        if (targetHeight == TargetHeight.High) {
            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.AlgaeRemoveHigh);
        }else{
            ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.AlgaeRemoveMiddle);
        }

        AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.IntakeReef);
        AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Intake);
    }

    @Override
    public void execute() {
        if (AlgaeSubsystem.getInstance().hasAlgae()) {
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Hold);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ElevatorSubsystem.getInstance().setElevatorState(ElevatorState.StoreAlgae);
        if (AlgaeSubsystem.getInstance().hasAlgae()) {
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Hold);
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Hold);
        }else{
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Off);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
