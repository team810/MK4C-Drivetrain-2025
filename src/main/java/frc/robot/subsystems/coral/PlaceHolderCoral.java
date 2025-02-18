package frc.robot.subsystems.coral;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PlaceHolderCoral implements CoralIO{
    public PlaceHolderCoral() {

    }

    @Override
    public void simulationPeriodic() {
        CoralIO.super.simulationPeriodic();
    }

    @Override
    public void writePeriodic() {
        CoralIO.super.writePeriodic();
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public DoubleSolenoid.Value getPistonState() {
        return null;
    }

    @Override
    public boolean hasCoral() {
        return false;
    }

    @Override
    public void setPistonState(DoubleSolenoid.Value value) {

    }

    @Override
    public void setVoltage(Voltage voltage) {

    }
}
