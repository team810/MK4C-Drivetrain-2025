package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {
    public void setElevator(Angle setpoint);
    public boolean atSetpoint();
    public Distance getHeight();

    public void readPeriodic();
    public void writePeriodic();
    public void simulationPeriodic();

}
