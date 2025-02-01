package frc.robot.subsystems.horn;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static frc.robot.subsystems.horn.HornConstants.*;

public class HornTalonFX implements HornIO{

    private final TalonFX motor;
    private final DoubleSolenoid piston;
    private final CANrange sensor;

    public HornTalonFX(){
        motor = new TalonFX(LASER_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(config);

        piston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                PISTON_FWD_ID, PISTON_REV_ID);

        sensor = new CANrange(LASER_ID);
    }

    @Override
    public void setState(HornState state) {
        switch(state){
            case Hold:
                motor.set(0);
                piston.set(DoubleSolenoid.Value.kReverse);
                break;
            case Intake:
                motor.set(-HornConstants.INTAKE_SPEED);
                piston.set(DoubleSolenoid.Value.kReverse);
                break;
            case Score:
                motor.set(HornConstants.SCORE_SPEED);
                piston.set(DoubleSolenoid.Value.kForward);
                break;
        }
    }

    @Override
    public boolean atSetpoint() {
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
