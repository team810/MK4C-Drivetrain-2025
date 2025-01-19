package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;

import java.util.HashMap;


public class ElevatorSubsystem extends AdvancedSubsystem {
    private final HashMap<ElevatorState, Double> elevatorHeights;
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

        if (Robot.isReal() && ElevatorConstants.ELEVATOR_ON_ROBOT) {
            elevator = new ElevatorTalonFX();
        }else{
            elevator = new ElevatorIOSim();
        }
    }

    public void setElevatorState(ElevatorState state) {
        this.currentState = state;
        elevator.setElevator(elevatorHeights.get(currentState));
    }

    @Logged (name = "Elevator Current State")
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
