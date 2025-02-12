package frc.robot.subsystems.algae;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;


public class    AlgaeConstants {
    public static final String CANBUS = "Mech";
    public static final int PIVOT_MOTOR_ID = 14;
    public static final int DRIVE_MOTOR_ID = 15;
    public static final int LASER_ID = 16;

    public static final Angle STORED_ANGLE = Degrees.of(80);
    public static final Angle BARGE_ANGLE = Degrees.of(60);
    public static final Angle HOLD_ANGLE = Degrees.of(0);
    public static final Angle PROCESSOR_ANGLE = Degrees.of(0);
    public static final Angle PIVOT_TOLERANCE = Degrees.of(.5);

    public static final Angle MAX_PIVOT = Degrees.of(90);
    public static final Angle MIN_PIVOT = Degrees.of(-20);

    public static final Distance LASER_TOLERANCE = Distance.ofBaseUnits(.05, Units.Meters);
    public static final Distance LASER_EXPECTED = Distance.ofBaseUnits(.05, Units.Meters);

    public static final Voltage INTAKE_VOLTAGE = Volts.of(12);
    public static final Voltage BARGE_VOLTAGE = Volts.of(-12);
    public static final Voltage PROCESSOR_VOLTAGE = Volts.of(-12);
    public static final Voltage HOLD_VOLTAGE = Volts.of(.5);


}
