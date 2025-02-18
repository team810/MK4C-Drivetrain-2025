package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class ElevatorConstants {
    public static final int PRIMARY_MOTOR_ID = 1;
    public static final int SECONDARY_MOTOR_ID = 2;

    public static final double CONVERSION_FACTOR = 1.0911 * 3; // rotations/in

    public static final Distance SOURCE_HEIGHT = Inches.of(20);
    public static final Distance L4_HEIGHT = Inches.of(70);
    public static final Distance L3_HEIGHT = Inches.of(50);
    public static final Distance L2_HEIGHT = Inches.of(30);
    public static final Distance TROUGH_HEIGHT = Inches.of(20);
    public static final Distance PROCESSOR_HEIGHT = Inches.of(5);
    public static final Distance ALGAE_HIGH_HEIGHT = Inches.of(70);
    public static final Distance ALGAE_MIDDLE_HEIGHT = Inches.of(50);
    public static final Distance BARGE_HEIGHT = Inches.of(75);
    public static final Distance STORE_CORAL_HEIGHT = Inches.of(5);
    public static final Distance STORE_ALGAE_HEIGHT = Inches.of(5);

    public static final Distance DRUM_RADIUS = Inches.of(.98);
    public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(90);

}
