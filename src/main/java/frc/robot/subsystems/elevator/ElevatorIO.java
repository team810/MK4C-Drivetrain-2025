package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {
    public void setElevator(double setpoint);
    public boolean atSetpoint();
    public double getHeight();

    public void readPeriodic();
    public void writePeriodic();
    public void simulationPeriodic();

}
