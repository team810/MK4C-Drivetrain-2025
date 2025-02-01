package frc.robot.subsystems.horn;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.littletonrobotics.junction.Logger;


public class HornSparkMax implements HornIO{
    private final SparkMax motor;
    private final DoubleSolenoid piston;
    private final CANrange sensor;
    private final StatusSignal<Distance> distanceStatusSignal;


    public HornSparkMax(){

        motor = new SparkMax(HornConstants.HORN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        piston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                                    HornConstants.PISTON_FWD_ID ,HornConstants.PISTON_REV_ID);

        sensor = new CANrange(HornConstants.LASER_ID);
        distanceStatusSignal = sensor.getDistance();
    }

    @Override
    public void setState(HornState state) {
        switch(state){
            case Hold:
                motor.set(0);
                piston.set(DoubleSolenoid.Value.kReverse);
                break;
            case Intake:
                motor.set(HornConstants.INTAKE_SPEED);
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
        return MathUtil.isNear(HornConstants.SENSOR_SETPOINT, distanceStatusSignal.getValue().in(Units.Meters),  HornConstants.SENSOR_TOLERANCE);
    }

    @Override
    public void readPeriodic() {
        Logger.recordOutput("Horn/SensorRaw", distanceStatusSignal.getValue().in(Units.Meters));
        Logger.recordOutput("Horn/CurrentPistonState",piston.get());
        Logger.recordOutput("Horn/MotorTemp", motor.getMotorTemperature());
        Logger.recordOutput("Horn/MotorCurrentDraw", motor.getOutputCurrent());
    }

    @Override
    public void writePeriodic() {


    }

    @Override
    public void simulationPeriodic() {

    }


}
