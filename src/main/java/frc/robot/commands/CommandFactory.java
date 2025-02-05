package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CommandFactory {

    public static Command getScoreAlgaeCommand() {
        return new InstantCommand();
    }
    public static Command getScoreCoralCommand() {
        return new InstantCommand();
    }

    public static Command PositionL4() {
        return new InstantCommand();
    }

    public static Command PositionL3() {
        return new InstantCommand();
    }

    public static Command PositionL2() {
        return new InstantCommand();
    }

    public static Command PositionTrough() {
        return new InstantCommand();
    }

    public static Command Source() {
        return new InstantCommand();
    }

    public static Command Processor() {
        return new InstantCommand();
    }

    public static Command PositionBarge() {
        return new InstantCommand();
    }

    public static Command StoreCoral() {
        return new InstantCommand();
    }

    public static Command StoreAlgae() {
        return new InstantCommand();
    }

}
