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

    public static final double STARTING_ANGLE = Math.PI/2;
    public static final double STORED_ANGLE = Math.PI/2;
    public static final double BARGE_ANGLE = Math.PI/2;
    public static final double HOLD_ANGLE = 0;
    public static final double PROCESSOR_ANGLE = 0;
    public static final double PIVOT_TOLERANCE = .01;

    public static final double MAX_PIVOT = 90;
    public static final double MIN_PIVOT = 0;

    public static final Distance LASER_TOLERANCE = Distance.ofBaseUnits(.05, Units.Meters);
    public static final Distance LASER_EXPECTED = Distance.ofBaseUnits(.05, Units.Meters);

    public static final Voltage INTAKE_VOLTAGE = Voltage.ofBaseUnits(5, Units.Volts);
    public static final Voltage BARGE_VOLTAGE = Voltage.ofBaseUnits(5, Units.Volts);
    public static final Voltage HOLD_VOLTAGE = Voltage.ofBaseUnits(5, Units.Volts);


}
