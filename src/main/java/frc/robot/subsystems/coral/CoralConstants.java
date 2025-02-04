package frc.robot.subsystems.coral;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

public class CoralConstants {
    public static final String CAN_BUS = "Mech";
    public static final int MOTOR_ID = 9;
    public static final int LASER_ID = 10;
    public static final int PISTON_FWD_CHANNEL = 2;
    public static final int PISTON_REV_CHANNEL= 3;

    public static final double LASER_EXPECTED = 1.0; // This is the expected output from the laser if the coral is detected
    public static final double LASER_TOLERANCE = 0.05;

    public static final Voltage SOURCE_VOLTAGE = Voltage.ofBaseUnits(0, Units.Volts);
    public static final Voltage REEF_SCORE_VOLTAGE = Voltage.ofBaseUnits(0, Units.Volts);
    public static final Voltage TROUGH_SCORE_VOLTAGE = Voltage.ofBaseUnits(0, Units.Volts);
}

