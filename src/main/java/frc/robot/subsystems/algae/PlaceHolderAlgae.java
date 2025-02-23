package frc.robot.subsystems.algae;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;

public class PlaceHolderAlgae implements AlgaeIO{
    public PlaceHolderAlgae()
    {

    }
    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {

    }

    @Override
    public void simulatePeriodic() {

    }

    @Override
    public boolean hasAlgae() {
        return false;
    }

    @Override
    public boolean atPivotSetpoint() {
        return false;
    }

    @Override
    public void setTargetPivot(Angle angle) {

    }

    @Override
    public void setDriveVoltage(VoltageOut voltage) {

    }

    @Override
    public double getCurrentPivot() {
        return 0;
    }
}
