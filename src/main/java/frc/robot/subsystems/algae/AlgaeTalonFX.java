package frc.robot.subsystems.algae;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

// CCW+ 0 is horizontal to the ground, 90 is straight up
public class AlgaeTalonFX implements AlgaeIO {
    private final TalonFX pivotMotor;
    private final MotionMagicVoltage pivotControl;
    private final SingleJointedArmSim pivotSim;
    private TalonFXSimState pivotSimState;

    private final StatusSignal<Angle> pivotPositionSignal;
    private final StatusSignal<AngularVelocity> pivotVelocitySignal;
    private final StatusSignal<Voltage> pivotVoltageSignal;
    private final StatusSignal<Temperature> pivotTemperatureSignal;
    private final StatusSignal<Current> pivotAppliedCurrentSignal;

    private double targetPivot;

    private final TalonFX driveMotor;
    private final VoltageOut driveVoltageControl;
    private Voltage driveAppliedVoltage;

    private final StatusSignal<Voltage> driveVoltageSignal;
    private final StatusSignal<Temperature> driveTemperatureSignal;
    private final StatusSignal<Current> driveAppliedCurrentSignal;

    private final CANrange laser;
    private final StatusSignal<Distance> distanceSignal;
    private final StatusSignal<Boolean> detectedSignal;

    public AlgaeTalonFX() {
        pivotMotor = new TalonFX(AlgaeConstants.PIVOT_MOTOR_ID, AlgaeConstants.CANBUS);
        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Takes the cos of the angle and then calculates the input needed to overcome gravity
        pivotMotorConfig.Slot0.kS = 0;
        pivotMotorConfig.Slot0.kV = .125; // Volts
        pivotMotorConfig.Slot0.kA = 0;
        pivotMotorConfig.Slot0.kG = 0;
        pivotMotorConfig.Slot0.kP = 64 * 5;
        pivotMotorConfig.Slot0.kI = 0;
        pivotMotorConfig.Slot0.kD = 0;

        pivotMotorConfig.Feedback.SensorToMechanismRatio = 64; // 64 to 1 gear box

        pivotMotorConfig.Voltage.PeakForwardVoltage = 12;
        pivotMotorConfig.Voltage.PeakReverseVoltage = -12;

        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 400;
        pivotMotorConfig.MotionMagic.MotionMagicJerk = 1000;

//        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaeConstants.MAX_PIVOT.in(Units.Rotations);
//        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaeConstants.MIN_PIVOT.in(Units.Rotations);

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        targetPivot = AlgaeConstants.STARTING_ANGLE;

        pivotControl = new MotionMagicVoltage(.25);
        pivotControl.EnableFOC = false;
        pivotControl.Slot = 0;
        pivotControl.LimitForwardMotion = false;
        pivotControl.LimitReverseMotion = false;
        pivotControl.UpdateFreqHz = 1000;

        pivotPositionSignal = pivotMotor.getPosition();
        pivotVelocitySignal = pivotMotor.getVelocity();
        pivotTemperatureSignal = pivotMotor.getDeviceTemp();
        pivotVoltageSignal = pivotMotor.getMotorVoltage();
        pivotAppliedCurrentSignal = pivotMotor.getSupplyCurrent();

        pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                64,
                .001,
                edu.wpi.first.math.util.Units.inchesToMeters(6),
                -Math.PI,
                Math.PI/2,
                true,
                0
        );
        pivotSim.setState(Math.PI/2,0);
        pivotMotor.setPosition(pivotSim.getAngleRads()/ (2 * Math.PI));
        pivotSimState = pivotMotor.getSimState();

        driveMotor = new TalonFX(AlgaeConstants.DRIVE_MOTOR_ID, AlgaeConstants.CANBUS);
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        driveMotorConfig.Voltage.PeakForwardVoltage = 12;
        driveMotorConfig.Voltage.PeakReverseVoltage = -12;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveMotorConfig);

        driveVoltageControl = new VoltageOut(0);
        driveVoltageControl.EnableFOC = false;
        driveVoltageControl.UpdateFreqHz = 1000;
        driveVoltageControl.UseTimesync = false;
        driveVoltageControl.LimitReverseMotion = false;
        driveVoltageControl.LimitForwardMotion = false;

        driveAppliedVoltage = Units.Volts.of(0);

        driveMotor.setControl(driveVoltageControl);

        driveVoltageSignal = driveMotor.getMotorVoltage();
        driveAppliedCurrentSignal = driveMotor.getSupplyCurrent();
        driveTemperatureSignal = driveMotor.getDeviceTemp();

        laser = new CANrange(AlgaeConstants.LASER_ID, AlgaeConstants.CANBUS);
        CANrangeConfiguration laserConfig = new CANrangeConfiguration();
        laserConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        laserConfig.FutureProofConfigs = true;
        laser.getConfigurator().apply(laserConfig);
        distanceSignal = laser.getDistance();
        detectedSignal = laser.getIsDetected();
    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(
                pivotPositionSignal,
                pivotVelocitySignal,
                pivotTemperatureSignal,
                pivotVoltageSignal,
                pivotAppliedCurrentSignal,

                distanceSignal,
                detectedSignal,

                driveVoltageSignal,
                driveAppliedCurrentSignal,
                driveTemperatureSignal
        );

        Logger.recordOutput("Algae/Pivot/TargetAngle",pivotControl.Position);
        Logger.recordOutput("Algae/Pivot/Position",pivotPositionSignal.getValue().in(Units.Rotations));
        Logger.recordOutput("Algae/Pivot/Velocity",pivotVelocitySignal.getValue());
        Logger.recordOutput("Algae/Pivot/MotorTemp", pivotTemperatureSignal.getValue());
        Logger.recordOutput("Algae/Pivot/Voltage", pivotVoltageSignal.getValue());
        Logger.recordOutput("Algae/Pivot/AppliedCurrent", pivotAppliedCurrentSignal.getValue());

        Logger.recordOutput("Algae/Laser/RawValue", distanceSignal.getValue());
        Logger.recordOutput("Algae/Laser/DetectedAlgae", hasAlgae());

        Logger.recordOutput("Algae/Drive/Voltage", driveVoltageSignal.getValue());
        Logger.recordOutput("Algae/Drive/Current", driveAppliedCurrentSignal.getValue());
        Logger.recordOutput("Algae/Drive/Temperature", driveTemperatureSignal.getValue());
        Logger.recordOutput("Algae/Drive/AppliedVoltage", driveAppliedVoltage);
    }

    @Override
    public void writePeriodic() {
        pivotMotor.setControl(pivotControl); // Apply pivot setpoint
        driveMotor.setControl(driveVoltageControl); // Applied drive voltage
    }

    @Override
    public void simulatePeriodic() {
        pivotSim.setInputVoltage(pivotVoltageSignal.getValue().in(Units.Volts));
        pivotSim.update(Robot.defaultPeriodSecs);

        pivotSimState = pivotMotor.getSimState();
        pivotSimState.setSupplyVoltage(12);
        pivotSimState.setRawRotorPosition((pivotSim.getAngleRads()/(2*Math.PI)) * 64);
        pivotSimState.setRotorVelocity((pivotSim.getAngleRads()/(2*Math.PI)) * 64);
    }

    @Override
    public boolean hasAlgae() {
        return MathUtil.isNear(
                AlgaeConstants.LASER_EXPECTED.in(Units.Meters),
                distanceSignal.getValue().in(Units.Meters),
                AlgaeConstants.LASER_TOLERANCE.in(Units.Meters)
        );
    }

    @Override
    public boolean atPivotSetpoint() {
        return MathUtil.isNear(
                targetPivot,
                pivotPositionSignal.getValue().in(Units.Radians),
                .001
        );
    }

    @Override
    public void setTargetPivot(double angle) {
        targetPivot = angle;
        pivotControl.Position = (angle / (2 * Math.PI));
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        driveAppliedVoltage = voltage;
        driveVoltageControl.Output = driveAppliedVoltage.in(Units.Volts);
    }

    @Override
    public double getCurrentPivot() {
        return pivotPositionSignal.getValue().in(Units.Radians);
    }
}
