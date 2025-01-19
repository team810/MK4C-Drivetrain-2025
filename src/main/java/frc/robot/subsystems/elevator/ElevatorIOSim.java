package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO{
    private final ElevatorSim elevatorsim;

    public ElevatorIOSim() {
        elevatorsim = new ElevatorSim(
                DCMotor.getKrakenX60(2),
                6,
                ElevatorConstants.CARRIAGE_MASS.in(Units.Kilogram),
                ElevatorConstants.DRUM_RADIUS_METERS.in(Units.Meters),
                ElevatorConstants.MIN_HEIGHT.in(Units.Meters),
                ElevatorConstants.MAX_HEIGHT.in(Units.Meters),
                true,
                ElevatorConstants.MIN_HEIGHT.in(Units.Meters),
                0
        );
    }
    @Override
    public void setElevator(double setpoint) {

    }

    @Override
    public boolean atSetpoint() {
        return false;
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {

    }
}
