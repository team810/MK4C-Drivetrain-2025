package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ElevatorTalonFX implements ElevatorIO{
    private final TalonFX leader;
    private final TalonFXSimState leaderSim;
    private final MotionMagicVoltage control;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;

    private Distance currentHeight = Distance.ofBaseUnits(0, Units.Meters);
    private LinearVelocity currentLinearVelocity = LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond);

    private final TalonFX follower;
    private final TalonFXSimState followerSim;
    private final Follower followerControl;

    private Distance targetHeight = Distance.ofBaseUnits(0, Units.Inch);

    private final StatusSignal<Temperature> leaderTempSignal;
    private final StatusSignal<Current> leaderCurrentSignal;
    private final StatusSignal<Voltage> leaderAppliedVoltageSignal;

    private final StatusSignal<Temperature> followerTempSignal;
    private final StatusSignal<Current> followerCurrentSignal;
    private final StatusSignal<Voltage> followerAppliedVoltageSignal;

    private ElevatorSim elevatorSim;

    public ElevatorTalonFX() {
        leader = new TalonFX(ElevatorConstants.PRIMARY_MOTOR_ID);
        leaderSim = leader.getSimState();
        follower = new TalonFX(ElevatorConstants.SECONDARY_MOTOR_ID);
        followerSim = follower.getSimState();

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

        leader.getConfigurator().apply(config);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        follower.getConfigurator().apply(config);

        positionSignal = leader.getPosition();
        velocitySignal = leader.getVelocity();

        leaderTempSignal = leader.getDeviceTemp();
        leaderCurrentSignal = leader.getSupplyCurrent();
        leaderAppliedVoltageSignal = leader.getMotorVoltage();

        followerTempSignal = follower.getDeviceTemp();
        followerCurrentSignal = follower.getSupplyCurrent();
        followerAppliedVoltageSignal = follower.getMotorVoltage();

        leader.setPosition(0);
        leader.setPosition(0);

        control = new MotionMagicVoltage(0);
        control.EnableFOC = false;
        control.Slot = 0;
        control.LimitReverseMotion = true;
        control.UpdateFreqHz = 1000;

        followerControl = new Follower(leader.getDeviceID(), false);
        followerControl.UpdateFreqHz = 1000;

        elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60(2),
                6,
                Mass.ofBaseUnits(35, Units.Pounds).in(Units.Kilograms),
                Distance.ofBaseUnits(.98, Units.Inch).in(Units.Meters),
                Distance.ofBaseUnits(0, Units.Meters).in(Units.Meters), // Min height
                Distance.ofBaseUnits(100, Units.Meters).in(Units.Meters), // Max height
                true, // Sim gravity
                Distance.ofBaseUnits(0, Units.Meters).in(Units.Meters), // Min height, // Starting height,
                0,
                0
        );
    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(positionSignal,
                velocitySignal,
                leaderAppliedVoltageSignal,
                leaderAppliedVoltageSignal,
                leaderTempSignal,
                followerAppliedVoltageSignal,
                followerTempSignal,
                followerCurrentSignal,
                followerAppliedVoltageSignal,
                followerTempSignal
        );

        currentHeight = Distance.ofBaseUnits(positionSignal.getValue().in(Units.Rotations) / ElevatorConstants.CONVERSION_FACTOR, Units.Inches);
        currentLinearVelocity = LinearVelocity.ofBaseUnits(velocitySignal.getValue().in(Units.RotationsPerSecond) / ElevatorConstants.CONVERSION_FACTOR, Units.InchesPerSecond);

        Logger.recordOutput("Elevator/CurrentHeight", currentHeight);
        Logger.recordOutput("Elevator/CurrentLinearVelocity", currentLinearVelocity);
        Logger.recordOutput("Elevator/AtTargetHeight", getHeight());

        Logger.recordOutput("Elevator/Leader/Voltage", leaderAppliedVoltageSignal.getValue().in(Units.Volts));
        Logger.recordOutput("Elevator/Leader/Current", leaderCurrentSignal.getValue().in(Units.Amps));
        Logger.recordOutput("Elevator/Leader/Temperature",leaderTempSignal.getValue().in(Units.Fahrenheit));

        Logger.recordOutput("Elevator/Follower/Voltage", followerAppliedVoltageSignal.getValue().in(Units.Volts));
        Logger.recordOutput("Elevator/Follower/Current", followerCurrentSignal.getValue().in(Units.Amps));
        Logger.recordOutput("Elevator/Follower/Temperature",followerTempSignal.getValue().in(Units.Fahrenheit));
    }

    @Override
    public void writePeriodic() {
        control.Position = targetHeight.in(Units.Inch) * ElevatorConstants.CONVERSION_FACTOR;
        leader.setControl(control);
        follower.setControl(followerControl);
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInputVoltage(leaderAppliedVoltageSignal.getValue().in(Units.Volts));
        elevatorSim.update(Robot.PERIOD);
        Angle position = Angle.ofBaseUnits(
                Distance.ofBaseUnits(
                        elevatorSim.getPositionMeters(), Units.Meters).in(Units.Inch) / ElevatorConstants.CONVERSION_FACTOR,
                Units.Rotations);

        leaderSim.setSupplyVoltage(12);
        leaderSim.setRawRotorPosition(position);
        followerSim.setSupplyVoltage(12);
        followerSim.setRawRotorPosition(position);
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
