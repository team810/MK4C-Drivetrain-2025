package frc.robot.subsystems.horn;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static frc.robot.subsystems.horn.HornConstants.*;

public class HornSparkMax implements HornIO{

    private final SparkMax motor;
    private final DoubleSolenoid piston;
    private final CANrange sensor;


    public HornSparkMax(){

        motor = new SparkMax(MOTOR_PORT, SparkLowLevel.MotorType.kBrushless);
        piston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                                    SOLENOID_PORT_1, SOLENOID_PORT_2);
        sensor = new CANrange(SENSOR_PORT);

    }

    @Override
    public void setState(HornState state) {
        switch(state){
            case Hold:
                motor.set(0);
                piston.set(DoubleSolenoid.Value.kReverse);
                break;
            case Intake:
                motor.set(-MOTOR_SPEED);
                piston.set(DoubleSolenoid.Value.kReverse);
                break;
            case Score:
                motor.set(MOTOR_SPEED);
                piston.set(DoubleSolenoid.Value.kForward);
                break;
        }
    }

    @Override
    public boolean atSetpoint() {
        double distance = sensor.getDistance().getValueAsDouble();
        return (distance < SENSOR_SETPOINT + SENSOR_TOLERANCE) && (distance > SENSOR_SETPOINT - SENSOR_TOLERANCE);
    }





}
