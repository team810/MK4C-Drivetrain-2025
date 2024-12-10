package frc.robot.IO;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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
    private static final Joystick secondaryJoystick  = new Joystick(1);

    private static final HashMap<Controls, Supplier<Double>> controlsJoystick = new HashMap<>();
    private static final HashMap<Controls, BooleanSupplier> controlsButtons = new HashMap<>();

    public static void Initialize(PrimaryDriverProfiles primaryProfile, SecondaryDriverProfiles secondaryProfile) {
        controlsJoystick.clear();
        controlsButtons.clear();

        switch(primaryProfile) {
            case Leo:
                controlsJoystick.put(Controls.driveXVelocity, primary::getLeftX);
                controlsJoystick.put(Controls.driveYVelocity, primary::getLeftY);
                controlsJoystick.put(Controls.driveOmega, primary::getRightX);

                controlsJoystick.put(Controls.driveThetaX, primary::getRightY);
                controlsJoystick.put(Controls.driveThetaY, primary::getRightX);

                controlsButtons.put(Controls.thetaOmegaToggle, primary::getRightStickButtonPressed);
                controlsButtons.put(Controls.resetGyro,primary::getAButton);

                controlsButtons.put(Controls.yawLock, () -> primary.getRightTriggerAxis() > .8);
                break;
        }

        switch (secondaryProfile) {
            case KnollJoystick:

                break;
            case KnollController:

                break;
        }
    }

    public static Supplier<Double> getJoystickValue(Controls control) {
        return controlsJoystick.get(control);
    }

    public static BooleanSupplier getButtonValue(Controls control) {
        return controlsButtons.get(control);
    }

    public static double getDPadPrimary()
    {
        return primary.getPOV();
    }

    public static void setPrimaryRumble()
    {
        primary.setRumble(GenericHID.RumbleType.kBothRumble, .6);
    }
}

