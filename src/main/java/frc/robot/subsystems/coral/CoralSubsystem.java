package frc.robot.subsystems.coral;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.lib.AdvancedSubsystem;

public class CoralSubsystem extends AdvancedSubsystem {
    private final CoralIO mech;
    private CoralState motorState;
    private DoubleSolenoid.Value pistonState;

    private CoralSubsystem() {
        mech = new CoralTalonFX();
    }

    public void setPistonState(DoubleSolenoid.Value pistonState) {
        this.pistonState = pistonState;
    }

    public void setCoralMotorState(CoralState motorState) {
        this.motorState = motorState;
        switch (motorState) {
            case Store -> {
                mech.motorOff();
                break;
            }
            case Hold -> {
                mech.motorHold();
                break;
            }
            case Intake -> {
                mech.motorIntake();
                break;
            }
            case Score -> {
                mech.motorScore();
                break;
            }
        }
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

    public CoralState getMotorState() {
        return motorState;
    }

    public boolean hasCoral(){
        return mech.hasCoral();
    }

    public DoubleSolenoid.Value getPistonState() {
        return pistonState;
    }
}

