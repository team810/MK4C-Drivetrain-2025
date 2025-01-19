package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants {

    public static final boolean ELEVATOR_ON_ROBOT = false;
    public static final int PRIMARY_MOTOR_ID = 1;
    public static final int SECONDARY_MOTOR_ID = 2;

    public static final Mass CARRIAGE_MASS = Mass.ofBaseUnits(1, Units.Pounds);
    public static final Distance DRUM_RADIUS_METERS = Distance.ofBaseUnits(0, Units.Meters);
    public static final Distance MIN_HEIGHT = Distance.ofBaseUnits(0, Units.Meter);
    public static final Distance MAX_HEIGHT = Distance.ofBaseUnits(1.2, Units.Meter);

}
