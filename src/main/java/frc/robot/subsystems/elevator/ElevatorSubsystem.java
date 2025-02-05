package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;


public class ElevatorSubsystem extends AdvancedSubsystem {
    private static ElevatorSubsystem instance;

    private final HashMap<ElevatorState, Distance> elevatorHeights; // Inches
    private ElevatorState currentState;
    private Distance targetHeight;

    private final ElevatorIO io;

    public ElevatorSubsystem() {
        currentState = ElevatorState.StoreCoral;
        elevatorHeights = new HashMap<>();

        elevatorHeights.put(ElevatorState.Source, ElevatorConstants.SOURCE_HEIGHT);
        elevatorHeights.put(ElevatorState.L4, ElevatorConstants.L4_HEIGHT);
        elevatorHeights.put(ElevatorState.L3, ElevatorConstants.L3_HEIGHT);
        elevatorHeights.put(ElevatorState.L2, ElevatorConstants.L2_HEIGHT);
        elevatorHeights.put(ElevatorState.Trough, ElevatorConstants.TROUGH_HEIGHT);
        elevatorHeights.put(ElevatorState.Processor, ElevatorConstants.PROCESSOR_HEIGHT);
        elevatorHeights.put(ElevatorState.AlgaeRemoveHigh, ElevatorConstants.ALGAE_HIGH_HEIGHT);
        elevatorHeights.put(ElevatorState.AlgaeRemoveMiddle, ElevatorConstants.ALGAE_MIDDLE_HEIGHT);
        elevatorHeights.put(ElevatorState.Barge, ElevatorConstants.BARGE_HEIGHT);
        elevatorHeights.put(ElevatorState.StoreCoral, ElevatorConstants.STORE_CORAL_HEIGHT);
        elevatorHeights.put(ElevatorState.StoreAlgae, ElevatorConstants.STORE_ALGAE_HEIGHT);

        targetHeight = elevatorHeights.get(currentState);

        io = new ElevatorTalonFX();

    }


    @Override
    public void readPeriodic() {
        Logger.recordOutput("Elevator/State", currentState);
        Logger.recordOutput("Elevator/Target", targetHeight);
        io.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        io.writePeriodic();
    }

    @Override
    public void simulatePeriodic() {
        io.simulationPeriodic();
    }

    public void setElevatorState(ElevatorState state) {
        this.currentState = state;
        targetHeight = elevatorHeights.get(this.currentState);
        io.setElevator(targetHeight);
    }

    public boolean atSetpoint() {
        return io.atSetpoint();
    }

    public ElevatorState getElevatorState() {return currentState;}

    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSubsystem();
        }
        return instance;
    }
}
