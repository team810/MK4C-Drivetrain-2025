package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class ElevatorTalonFX implements ElevatorIO{
    private final TalonFX primary;
    private final TalonFX secondary;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;

    private Distance targetHeight = Distance.ofBaseUnits(0, Units.Inch);
    private final MotionMagicVoltage control;
    private final Follower slaveControl;

    public ElevatorTalonFX() {
        primary = new TalonFX(ElevatorConstants.PRIMARY_MOTOR_ID);
        secondary = new TalonFX(ElevatorConstants.SECONDARY_MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        // Current config
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // Voltage config
        config.Voltage.PeakForwardVoltage = -12;
        config.Voltage.PeakReverseVoltage = -12;
        // Motion config
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kG = 0;
        config.Slot0.kV = .124;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        // Configure pid controller
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100; // Test to set the upper limit

        config.MotionMagic.MotionMagicCruiseVelocity = 50;
        config.MotionMagic.MotionMagicAcceleration = 50;
        config.MotionMagic.MotionMagicJerk = 1000;

        primary.getConfigurator().apply(config);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        secondary.getConfigurator().apply(config);

        positionSignal = primary.getPosition();
        velocitySignal = primary.getVelocity();
        voltageSignal = primary.getMotorVoltage();

        primary.setPosition(0);
        secondary.setPosition(0);

        control = new MotionMagicVoltage(0);
        control.EnableFOC = true;
        control.Slot = 0;
        control.LimitReverseMotion = true;
        control.UpdateFreqHz = 1000;

        slaveControl = new Follower(primary.getDeviceID(), false);
        slaveControl.UpdateFreqHz = 1000;
    }

    @Override
    public void readPeriodic() {
        Logger.recordOutput("Elevator/Height", getHeight());
        Logger.recordOutput("Elevator/Velocity", velocitySignal.getValue().in(Units.RotationsPerSecond)/ElevatorConstants.CONVERSION_FACTOR);
        Logger.recordOutput("Elevator/VoltageOutput", voltageSignal.getValue().in(Units.Volts));
        Logger.recordOutput("Elevator/TargetHeight", targetHeight.in(Units.Inches));
        Logger.recordOutput("Elevator/AtTargetHeight", targetHeight.in(Units.Inches));
    }

    @Override
    public void writePeriodic() {
        control.Position = targetHeight.in(Units.Inch) * ElevatorConstants.CONVERSION_FACTOR;
        primary.setControl(control);

        secondary.setControl(slaveControl);
    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void setElevator(Distance targetHeight) {
        this.targetHeight = targetHeight;
    }

    @Override
    public Distance getHeight() {
        double rotations = positionSignal.getValue().in(Units.Rotations);
        return Distance.ofBaseUnits(rotations/ElevatorConstants.CONVERSION_FACTOR, Units.Inch);
    }

    @Override
    public boolean atSetpoint() {
        return MathUtil.isNear(targetHeight.in(Units.Inch), getHeight().in(Units.Inch), .125);
    }
}
