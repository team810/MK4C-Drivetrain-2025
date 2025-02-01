package frc.robot.subsystems.coral;

public class CoralConstants {
    public static final String CAN_BUS = "Mech";
    public static final int MOTOR_ID = 0;
    public static final int LASER_ID = 1;
    public static final int PISTON_FWD_CHANNEL = 2;
    public static final int PISTON_REV_CHANNEL= 3;

    public static final double LASER_EXPECTED = 1.0; // This is the expected output from the laser if the coral is detected
    public static final double LASER_TOLERANCE = 0.05;

    public static final double INTAKE_VOLTAGE = 1.0; // Source intake speeds Percent -12 : 12
    public static final double SCORE_VOLTAGE = 1; // Score speeds -12 : 12
}

