package frc.robot.subsystems.coral;

public interface CoralIO {
    public void setState(CoralState state);
    public boolean hasCoral();

    public void readPeriodic();
    public void writePeriodic();
    public void simulationPeriodic();

}
