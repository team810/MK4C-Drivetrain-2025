package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {
    public void setElevator(Distance setpoint);
    public boolean atSetpoint();
    public Distance getHeight();

    public void readPeriodic();
    public void writePeriodic();
    public void simulationPeriodic();

}
