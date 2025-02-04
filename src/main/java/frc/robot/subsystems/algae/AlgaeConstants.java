package frc.robot.subsystems.algae;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class AlgaeConstants {
    public static final String CANBUS = "Mech";
    public static final int PIVOT_MOTOR_ID = 1;
    public static final int DRIVE_MOTOR_ID = 2;
    public static final int LASER_ID = 3;

    public static final Angle STARTING_ANGLE = Angle.ofBaseUnits(90, Units.Degrees);

    public static final Angle MAX_PIVOT = Angle.ofBaseUnits(90, Units.Degrees);
    public static final Angle MIN_PIVOT = Angle.ofBaseUnits(0, Units.Degrees);

    public static final Distance LASER_TOLERANCE = Distance.ofBaseUnits(.05, Units.Meters);
    public static final Distance LASER_EXPECTED = Distance.ofBaseUnits(.05, Units.Meters);

}
