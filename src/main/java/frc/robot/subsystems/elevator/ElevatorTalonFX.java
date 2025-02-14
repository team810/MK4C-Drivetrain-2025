package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ElevatorTalonFX implements ElevatorIO{
    private final TalonFX leader;
    private TalonFXSimState leaderSim;

    private final TalonFX follower;
    private TalonFXSimState followerSim;

    private final MotionMagicVoltage control;
    private final Follower followerControl;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;

    private final StatusSignal<Temperature> leaderTempSignal;
    private final StatusSignal<Voltage> leaderAppliedVoltageSignal;
    private final StatusSignal<Current> leaderSupplyCurrentSignal;
    private final StatusSignal<Current> leaderAppliedCurrentSignal;

    private final StatusSignal<Temperature> followerTempSignal;
    private final StatusSignal<Current> followerCurrentSignal;
    private final StatusSignal<Voltage> followerAppliedVoltageSignal;

    private final ElevatorSim elevatorSim;

    private Distance currentHeight;
    private Distance targetHeight;

    public ElevatorTalonFX() {
        leader = new TalonFX(ElevatorConstants.PRIMARY_MOTOR_ID);
        leaderSim = leader.getSimState();
        follower = new TalonFX(ElevatorConstants.SECONDARY_MOTOR_ID);
        followerSim = follower.getSimState();

        TalonFXConfiguration config = new TalonFXConfiguration();
        // Current config
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        // Voltage config
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        // Motion config
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        if (Robot.isReal()) {
            config.Slot0.kG = .5;
            config.Slot0.kP = 0;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;

            config.MotionMagic.MotionMagicCruiseVelocity = 50;
            config.MotionMagic.MotionMagicAcceleration = 100;
            config.MotionMagic.MotionMagicJerk = 1000;
        }else{
            config.Slot0.kG = .5;
            config.Slot0.kP = 2;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;

            config.MotionMagic.MotionMagicCruiseVelocity = 100;
            config.MotionMagic.MotionMagicAcceleration = 250;
            config.MotionMagic.MotionMagicJerk = 1000;
        }

        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

        positionSignal = leader.getPosition();
        velocitySignal = leader.getVelocity();

        leaderTempSignal = leader.getDeviceTemp();
        leaderAppliedVoltageSignal = leader.getMotorVoltage();
        leaderAppliedCurrentSignal = leader.getStatorCurrent();
        leaderSupplyCurrentSignal = leader.getSupplyCurrent();

        followerTempSignal = follower.getDeviceTemp();
        followerCurrentSignal = follower.getSupplyCurrent();
        followerAppliedVoltageSignal = follower.getMotorVoltage();

        control = new MotionMagicVoltage(0);
        control.EnableFOC = true;
        control.Slot = 0;
        control.UseTimesync = false;

        followerControl = new Follower(leader.getDeviceID(), false);
        followerControl.UpdateFreqHz = 1000;

        leader.setPosition(0,3);

        targetHeight = ElevatorConstants.STORE_CORAL_HEIGHT;
        currentHeight = Inches.of(positionSignal.getValue().in(Rotations) / ElevatorConstants.CONVERSION_FACTOR);

        elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60Foc(2),
                6,
                Pounds.of(25).in(Kilograms),
                ElevatorConstants.DRUM_RADIUS.in(Meters),
                0,// Min height
                ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meters),
                true, // Sim gravity
                .01,
                0.000000001,
                0
        );
    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(positionSignal,
                velocitySignal,
                leaderAppliedVoltageSignal,
                leaderSupplyCurrentSignal,
                leaderAppliedCurrentSignal,
                leaderTempSignal,
                followerAppliedVoltageSignal,
                followerTempSignal,
                followerCurrentSignal,
                followerAppliedVoltageSignal,
                followerTempSignal
        );

        currentHeight = Inches.of(positionSignal.getValue().in(Rotations) / ElevatorConstants.CONVERSION_FACTOR);

        Logger.recordOutput("Elevator/CurrentHeightInches", currentHeight.in(Inches));
        Logger.recordOutput("Elevator/TargetHeightInches", targetHeight.in(Inches));
        Logger.recordOutput("Elevator/AtTargetHeight", atSetpoint());

        Logger.recordOutput("Elevator/TargetRaw", control.Position);
        Logger.recordOutput("Elevator/RawEncoder", positionSignal.getValue().in(Rotations));

        Logger.recordOutput("Elevator/Leader/Voltage", leaderAppliedVoltageSignal.getValue().in(Units.Volts));
        Logger.recordOutput("Elevator/Leader/AppliedCurrent", leaderAppliedCurrentSignal.getValue().in(Units.Amps));
        Logger.recordOutput("Elevator/Leader/SupplyCurrent", leaderSupplyCurrentSignal.getValue().in(Amps));
        Logger.recordOutput("Elevator/Leader/Temperature",leaderTempSignal.getValue().in(Units.Fahrenheit));

        Logger.recordOutput("Elevator/Follower/Voltage", followerAppliedVoltageSignal.getValue().in(Units.Volts));
        Logger.recordOutput("Elevator/Follower/Current", followerCurrentSignal.getValue().in(Units.Amps));
        Logger.recordOutput("Elevator/Follower/Temperature",followerTempSignal.getValue().in(Units.Fahrenheit));
    }

    @Override
    public void writePeriodic() {
        control.Position = targetHeight.in(Inches) * ElevatorConstants.CONVERSION_FACTOR;
        leader.setControl(control);
        follower.setControl(followerControl);
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInputVoltage(leaderAppliedVoltageSignal.getValue().in(Units.Volts));
        elevatorSim.update(Robot.PERIOD);

        double heightIn = edu.wpi.first.math.util.Units.metersToInches(elevatorSim.getPositionMeters());
        double velocityIn = edu.wpi.first.math.util.Units.metersToInches(elevatorSim.getVelocityMetersPerSecond());
        leaderSim = leader.getSimState();
        leaderSim.setSupplyVoltage(12);
        leaderSim.setRawRotorPosition(heightIn / ElevatorConstants.CONVERSION_FACTOR);
        leaderSim.setRotorVelocity(velocityIn/ ElevatorConstants.CONVERSION_FACTOR);
        followerSim = follower.getSimState();
        followerSim.setSupplyVoltage(12);
        followerSim.setRawRotorPosition(heightIn / ElevatorConstants.CONVERSION_FACTOR);
        followerSim.setRotorVelocity(velocityIn/ ElevatorConstants.CONVERSION_FACTOR);
    }

    @Override
    public void setElevator(Distance targetHeight) {
        this.targetHeight = targetHeight;
    }

    @Override
    public Distance getHeight() {
        return currentHeight;
    }

    @Override
    public boolean atSetpoint() {
        return MathUtil.isNear(targetHeight.in(Inches),currentHeight.in(Inches), .125);
    }
}
