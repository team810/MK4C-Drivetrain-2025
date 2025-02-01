package frc.robot.subsystems.coral;


import frc.robot.lib.AdvancedSubsystem;

public class CoralSubsystem extends AdvancedSubsystem {
    private final CoralIO mech;
    private CoralState mechState;

    private CoralSubsystem() {
        mech = new CoralTalonFX();
    }

    public void setState(CoralState mechState) {
        this.mechState = mechState;
        mech.setState(mechState);
    }

    public CoralState getState(){
        return mechState;
    }

    public boolean hasCoral(){
        return mech.hasCoral();
    }

    @Override
    public void readPeriodic() {
        mech.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        mech.writePeriodic();
    }

    @Override
    public void simulatePeriodic() {
        mech.simulationPeriodic();
    }
}

