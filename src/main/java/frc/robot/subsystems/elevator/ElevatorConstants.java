package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    public static final int PRIMARY_MOTOR_ID = 10;
    public static final int SECONDARY_MOTOR_ID = 11;

    public static final double CONVERSION_FACTOR = 1.0911; // rotations/in

    public static final Distance SOURCE_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance L4_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance L3_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance L2_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance TROUGH_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance PROCESSOR_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance ALGAE_HIGH_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance ALGAE_MIDDLE_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance BARGE_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance STORE_CORAL_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);
    public static final Distance STORE_ALGAE_HEIGHT = Distance.ofBaseUnits(0, Units.Inches);

}
