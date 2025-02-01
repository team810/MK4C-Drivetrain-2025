package frc.robot.subsystems.horn;


import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;

public class HornSubsystem extends AdvancedSubsystem {

    private final HornIO horn;
    private HornState hornState;

    private HornSubsystem() {

        if(!Robot.isReal())
            horn = new HornIOSim();
        else if(HornConstants.IS_TALONFX)
            horn = new HornTalonFX();
        else
            horn = new HornSparkMax();
    }

    public void setState(HornState hornState){
        this.hornState = hornState;
        horn.setState(hornState);
    }

    public HornState getState(){
        return hornState;
    }

    public boolean atSetpoint(){
        return horn.atSetpoint();
    }

    @Override
    public void readPeriodic() {
        horn.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        horn.writePeriodic();
    }

    @Override
    public void simulatePeriodic() {
        horn.simulationPeriodic();
    }
}

