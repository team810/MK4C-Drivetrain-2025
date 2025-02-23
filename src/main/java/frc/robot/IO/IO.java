package frc.robot.IO;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public abstract class IO {
    public enum PrimaryDriverProfiles {
        Leo,
    }
    public enum SecondaryDriverProfiles {
        KnollJoystick,
        KnollController
    }

    private static final XboxController primary = new XboxController(0);
    private static final XboxController secondary = new XboxController(1);

    private static final HashMap<Controls, DoubleSupplier> controlsJoystick = new HashMap<>();
    private static final HashMap<Controls, BooleanSupplier> controlsButtons = new HashMap<>();

    public static void Initialize(PrimaryDriverProfiles primaryProfile, SecondaryDriverProfiles secondaryProfile) {
        controlsJoystick.clear();
        controlsButtons.clear();

        switch(primaryProfile) {
            case Leo:
                controlsJoystick.put(Controls.driveXVelocity, primary::getLeftX);
                controlsJoystick.put(Controls.driveYVelocity, primary::getLeftY);
                controlsJoystick.put(Controls.driveOmega, primary::getRightX);
                controlsButtons.put(Controls.resetGyro,primary::getAButton);

                controlsButtons.put(Controls.leftAlign,() -> primary.getLeftTriggerAxis() > .8);
                controlsButtons.put(Controls.rightAlign, () -> primary.getRightTriggerAxis() > .8);

                controlsButtons.put(Controls.leftSource, primary::getLeftBumperButton);
                controlsButtons.put(Controls.rightSource, primary::getRightBumperButton);
                break;
        }

        controlsButtons.put(Controls.PositionL4, secondary::getYButton);
        controlsButtons.put(Controls.PositionL3, secondary::getXButton);
        controlsButtons.put(Controls.PositionL2, secondary::getBButton);
        controlsButtons.put(Controls.Store, secondary::getAButton);

        controlsButtons.put(Controls.AlgaeIntakeReefHigh, secondary::getLeftBumperButton);
        controlsButtons.put(Controls.AlgaeIntakeReefLow, secondary::getRightBumperButton);
        controlsButtons.put(Controls.AlgaeIntakeCoral, () -> secondary.getRightTriggerAxis() > .8);
        controlsButtons.put(Controls.AlgaeIntakeGround, () -> secondary.getLeftTriggerAxis() > .8);

//        switch (secondaryProfile) {
//            case KnollJoystick:
//
//                break;
//            case KnollController:
//                break;
//        }
    }

    public static DoubleSupplier getJoystickValue(Controls control) {
        return controlsJoystick.get(control);
    }

    public static BooleanSupplier getButtonValue(Controls control) {
        return controlsButtons.get(control);
    }

    public static double getDPadPrimary() {
        return primary.getPOV();
    }

    public static void setPrimaryRumble()
    {
        primary.setRumble(GenericHID.RumbleType.kBothRumble, .6);
    }
}

