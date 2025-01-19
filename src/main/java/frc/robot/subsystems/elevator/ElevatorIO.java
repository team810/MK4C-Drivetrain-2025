package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public void setElevator(double setpoint);
    public boolean atSetpoint();
    public void readPeriodic();
    public void writePeriodic();

}
