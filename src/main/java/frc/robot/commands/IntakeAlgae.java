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
        AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.IntakeGround);
    }

    @Override
    public void end(boolean interrupted) {
        if (AlgaeSubsystem.getInstance().hasAlgae()) {
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Hold);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Hold);
        }else{
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Off);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
        }
    }

    @Override
    public boolean isFinished() {
        return AlgaeSubsystem.getInstance().hasAlgae();
    }
}
