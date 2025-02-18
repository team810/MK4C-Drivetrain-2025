package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeDriveStates;
import frc.robot.subsystems.algae.AlgaePivotStates;
import frc.robot.subsystems.algae.AlgaeSubsystem;

public class ScoreAlgae extends Command {
    @Override
    public void initialize() {
        if (AlgaeSubsystem.getInstance().getPivotState() == AlgaePivotStates.Processor) {
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Processor);
        }else {
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Barge);
        }
    }
    @Override
    public void end(boolean interrupted) {
        AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Off);
        AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
    }

    @Override
    public boolean isFinished() {
        return !AlgaeSubsystem.getInstance().hasAlgae();
    }
}
