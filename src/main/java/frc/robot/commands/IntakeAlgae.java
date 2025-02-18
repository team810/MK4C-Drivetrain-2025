package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeDriveStates;
import frc.robot.subsystems.algae.AlgaePivotStates;
import frc.robot.subsystems.algae.AlgaeSubsystem;

public class IntakeAlgae extends Command {
    public IntakeAlgae() {

    }

    @Override
    public void initialize() {
        AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Intake);
        AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Processor);
    }

    @Override
    public void end(boolean interrupted) {
        AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Hold);
        AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Hold);
    }

    @Override
    public boolean isFinished() {
        return AlgaeSubsystem.getInstance().hasAlgae();
    }
}
