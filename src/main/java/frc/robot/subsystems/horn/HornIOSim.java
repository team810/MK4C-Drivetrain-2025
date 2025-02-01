package frc.robot.subsystems.horn;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.sim.CANrangeSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static frc.robot.subsystems.horn.HornConstants.*;

public class HornIOSim implements HornIO{

    private final FlywheelSim flywheelSim;
    private final DoubleSolenoidSim solenoidSim;
    private final CANrange sensor;
    private final CANrangeSimState sensorSim;


    public HornIOSim(){

        flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),WHEEL_INERTIA,6),
                DCMotor.getKrakenX60(1));
        solenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH,
                                            PISTON_FWD_ID,
                                            PISTON_REV_ID);
        sensor = new CANrange(LASER_ID);
        sensorSim = new CANrangeSimState(sensor);

    }


    @Override
    public void setState(HornState state) {
        switch(state){
            case Hold:
                flywheelSim.setInput(0);
                solenoidSim.set(DoubleSolenoid.Value.kReverse);
                break;
            case Intake:
                flywheelSim.setInput(-INTAKE_SPEED);
                solenoidSim.set(DoubleSolenoid.Value.kReverse);
                break;
            case Score:
                flywheelSim.setInput(SCORE_SPEED);
                solenoidSim.set(DoubleSolenoid.Value.kForward);
                break;
        }
    }

    @Override
    public boolean atSetpoint() {
        sensorSim.setDistance(2*Math.random()*SENSOR_SETPOINT);
        double distance = sensor.getDistance().getValueAsDouble();
        return (distance < SENSOR_SETPOINT + SENSOR_TOLERANCE) && (distance > SENSOR_SETPOINT - SENSOR_TOLERANCE);
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {

    }

    @Override
    public void simulationPeriodic() {

    }


}
