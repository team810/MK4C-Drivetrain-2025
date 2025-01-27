package frc.robot.subsystems.horn;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class HornSubsystem extends SubsystemBase {

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
}

