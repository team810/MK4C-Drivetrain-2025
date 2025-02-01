package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface CoralIO {
    public void setPistonState(DoubleSolenoid.Value value);
    public DoubleSolenoid.Value getPistonState();
    public boolean hasCoral();

    public void motorIntake();
    public void motorScore();
    public void motorHold();
    public void motorOff();

    public void readPeriodic();
    public default void writePeriodic() {return;};
    public default void simulationPeriodic() {return;};

}
