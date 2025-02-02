package frc.robot.subsystems.elevator;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.AdvancedSubsystem;

import java.util.HashMap;


public class ElevatorSubsystem extends AdvancedSubsystem {
    private final HashMap<ElevatorState, Double> elevatorHeights; // Inches
    private final ElevatorIO elevator;
    private ElevatorState currentState;

    public ElevatorSubsystem() {
        elevatorHeights = new HashMap<>();

        elevatorHeights.put(ElevatorState.Ground, 0.0);
        elevatorHeights.put(ElevatorState.Source, 0.0);
        elevatorHeights.put(ElevatorState.Processor, 0.0);
        elevatorHeights.put(ElevatorState.HighCoral, 0.0);
        elevatorHeights.put(ElevatorState.MiddleCoral, 0.0);
        elevatorHeights.put(ElevatorState.LowCoral, 0.0);
        elevatorHeights.put(ElevatorState.Trough, 0.0);
        elevatorHeights.put(ElevatorState.Barge,0.0);

        elevator = new ElevatorTalonFX();
    }

    public void setElevatorState(ElevatorState state) {
        this.currentState = state;
        elevator.setElevator(Distance.ofBaseUnits(elevatorHeights.get(currentState), Units.Inch));
    }

    public ElevatorState getElevatorState() {return currentState;}

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {

    }

    @Override
    public void simulatePeriodic() {

    }
}
