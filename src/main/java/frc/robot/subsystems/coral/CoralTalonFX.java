package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.littletonrobotics.junction.Logger;

public class CoralTalonFX implements CoralIO {

    private final TalonFX motor;
    private final VoltageOut motorControl;
    private final CANrange sensor;

    private final StatusSignal<Distance> laserDistance;
    private final StatusSignal<Boolean> laserIsDetected;
    private final DoubleSolenoid piston;

    private CoralState appliedMotorState;

    public CoralTalonFX() {
        motor = new TalonFX(CoralConstants.MOTOR_ID, CoralConstants.CAN_BUS);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);

        motorControl = new VoltageOut(0);
        motorControl.EnableFOC = true;

        sensor = new CANrange(CoralConstants.LASER_ID, CoralConstants.CAN_BUS);
        laserDistance = sensor.getDistance();
        laserIsDetected = sensor.getIsDetected();

        piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, CoralConstants.PISTON_FWD_CHANNEL, CoralConstants.PISTON_REV_CHANNEL);
    }

    @Override
    public void readPeriodic() {
        Logger.recordOutput("Coral/HasCoral", hasCoral());
        Logger.recordOutput("Coral/RawDistance", laserDistance.getValue());
        Logger.recordOutput("Coral/IsDetected", laserIsDetected.getValue());

        Logger.recordOutput("Coral/MotorTemp", motor.getDeviceTemp().getValue().in(Units.Fahrenheit));
        Logger.recordOutput("Coral/MotorVoltage", motor.getMotorVoltage().getValue().in(Units.Volts));
    }

    @Override
    public void motorIntake() {
        motorControl.Output = CoralConstants.INTAKE_VOLTAGE;
        motor.setControl(motorControl);
    }

    @Override
    public void motorScore() {
        motorControl.Output = CoralConstants.SCORE_VOLTAGE;
        motor.setControl(motorControl);
    }

    @Override
    public void motorHold() {
        motorControl.Output = CoralConstants.HOLD_VOLTAGE;
        motor.setControl(motorControl);
    }

    @Override
    public void motorOff() {
        motorControl.Output = 0;
        motor.setControl(motorControl);
    }

    @Override
    public void setPistonState(DoubleSolenoid.Value state) {
        piston.set(state);
    }

    @Override
    public boolean hasCoral() {
        if (laserIsDetected.getValue())
            return false;
        return MathUtil.isNear(
                CoralConstants.LASER_EXPECTED,
                laserDistance.getValue().in(Units.Meters),
                CoralConstants.LASER_TOLERANCE
        );
    }

    @Override
    public DoubleSolenoid.Value getPistonState() {
        return piston.get();
    }
}
