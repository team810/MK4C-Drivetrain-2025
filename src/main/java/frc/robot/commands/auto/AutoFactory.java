package frc.robot.commands.auto;

import java.util.HashMap;

public class AutoFactory {

    public enum StartOptions {
        Right,
        Center,
        Left,
    }

    public enum ReefOptions {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
    }

    public enum SourceOptions {
        Left,
        Right
    }

    private final HashMap<SourceOptions, String> sourceNames = new HashMap<SourceOptions, String>(); // Source, name
    private final HashMap<ReefOptions, String> reefNames = new HashMap<ReefOptions, String>(); // Reef, name
    private final HashMap<StartOptions, String> startNames = new HashMap<StartOptions, String>(); // start, name

}
