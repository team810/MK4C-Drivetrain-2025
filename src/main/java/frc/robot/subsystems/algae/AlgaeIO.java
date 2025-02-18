package frc.robot.subsystems.algae;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;

public interface AlgaeIO {
    public void readPeriodic();
    public void writePeriodic();
    public void simulatePeriodic();

    public boolean hasAlgae();
    public boolean atPivotSetpoint();
    public void setTargetPivot(Angle angle);
    public void setDriveVoltage(VoltageOut voltage);
    public double getCurrentPivot();

}
