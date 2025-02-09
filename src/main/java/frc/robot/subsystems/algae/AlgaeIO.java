package frc.robot.subsystems.algae;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface AlgaeIO {
    public void readPeriodic();
    public void writePeriodic();
    public void simulatePeriodic();

    public boolean hasAlgae();
    public boolean atPivotSetpoint();
    public void setTargetPivot(double angle);
    public void setDriveVoltage(Voltage voltage);
    public double getCurrentPivot();
}
