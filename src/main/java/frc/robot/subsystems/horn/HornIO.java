package frc.robot.subsystems.horn;

public interface HornIO {
    void setState(HornState state);
    boolean atSetpoint();
}
