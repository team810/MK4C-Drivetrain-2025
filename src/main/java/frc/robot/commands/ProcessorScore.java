package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeDriveStates;
import frc.robot.subsystems.algae.AlgaePivotStates;
import frc.robot.subsystems.algae.AlgaeSubsystem;

public class ProcessorScore extends Command {
    private int i;
    private boolean on;
    public ProcessorScore() {
        i = 0;
    }
    @Override
    public void initialize() {
        AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Processor);
        AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Processor);
    }

    @Override
    public void execute() {
        i++;
        if (i == 2) {
            if (on)
            {
                AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Off);
                on = false;
            }else{
                on = true;
                AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Processor);
            }
            i = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (AlgaeSubsystem.getInstance().hasAlgae()) {
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Hold);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Hold);
        }else {
            AlgaeSubsystem.getInstance().setDriveState(AlgaeDriveStates.Off);
            AlgaeSubsystem.getInstance().setPivotState(AlgaePivotStates.Stored);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
