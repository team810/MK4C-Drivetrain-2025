package frc.robot.subsystems.horn;

public interface HornIO {
    public void setState(HornState state);
    public boolean atSetpoint();

    public void readPeriodic();
    public void writePeriodic();
    public void simulationPeriodic();

}
