package frc.robot.subsystems.algae;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class AlgaeConstants {
    public static final String CANBUS = "Mech";
    public static final int PIVOT_MOTOR_ID = 14;
    public static final int DRIVE_MOTOR_ID = 15;
    public static final int LASER_ID = 16;

    public static final Angle STARTING_ANGLE = Angle.ofBaseUnits(90, Units.Degrees);
    public static final Angle STORED_ANGLE = Angle.ofBaseUnits(90, Units.Degrees);
    public static final Angle BARGE_ANGLE = Angle.ofBaseUnits(90, Units.Degrees);
    public static final Angle HOLD_ANGLE = Angle.ofBaseUnits(90, Units.Degrees);
    public static final Angle PROCESSOR_ANGLE = Angle.ofBaseUnits(90, Units.Degrees);
    public static final Angle PIVOT_TOLERANCE = Angle.ofBaseUnits(.05, Units.Degrees);

    public static final Angle MAX_PIVOT = Angle.ofBaseUnits(90, Units.Degrees);
    public static final Angle MIN_PIVOT = Angle.ofBaseUnits(0, Units.Degrees);

    public static final Distance LASER_TOLERANCE = Distance.ofBaseUnits(.05, Units.Meters);
    public static final Distance LASER_EXPECTED = Distance.ofBaseUnits(.05, Units.Meters);

    public static final Voltage INTAKE_VOLTAGE = Voltage.ofBaseUnits(5, Units.Volts);
    public static final Voltage BARGE_VOLTAGE = Voltage.ofBaseUnits(5, Units.Volts);
    public static final Voltage HOLD_VOLTAGE = Voltage.ofBaseUnits(5, Units.Volts);


}
