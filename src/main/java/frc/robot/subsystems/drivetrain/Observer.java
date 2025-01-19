package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static edu.wpi.first.units.Units.*;

public class Observer extends Thread {
    private final ArrayList<SwerveObservation> observations;
    private final ReadWriteLock observationsLock;

    private final ReadWriteLock moduleObservationLock;

    private ModuleObservationRaw frontLeftObservation;
    private final ModuleSignals frontLeftSignals;

    private ModuleObservationRaw frontRightObservation;
    private final ModuleSignals frontRightSignals;

    private ModuleObservationRaw backLeftObservation;
    private final ModuleSignals backLeftSignals;

    private ModuleObservationRaw backRightObservation;
    private final ModuleSignals backRightSignals;

    private final ReadWriteLock yawLock;
    private final StatusSignal<Angle> yawSignal;

    public static class SwerveObservation {
        SwerveModulePosition frontLeft;
        SwerveModulePosition frontRight;
        SwerveModulePosition backLeft;
        SwerveModulePosition backRight;

        Rotation2d yaw;

        double timestamp;

        public SwerveObservation(SwerveModulePosition frontLeft, SwerveModulePosition frontRight, SwerveModulePosition backLeft, SwerveModulePosition backRight, Rotation2d yaw, double timestamp) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
            this.yaw = yaw;
            this.timestamp = timestamp;
        }
    }

    public static class ModuleObservationRaw {
        public Measure<AngleUnit> position;
        public Measure<AngularVelocityUnit> velocity;
        public Measure<AngularAccelerationUnit> acceleration;
        public Measure<CurrentUnit> current;
        public Measure<VoltageUnit> appliedVoltage;
        public Measure<AngleUnit> theta;
        public Measure<AngularVelocityUnit> omega;

        public ModuleObservationRaw() {
            position = Angle.ofBaseUnits(0,Radians);
            velocity = AngularVelocity.ofBaseUnits(0, RadiansPerSecond);
            acceleration = AngularAcceleration.ofBaseUnits(0, RadiansPerSecondPerSecond);
            current = Current.ofBaseUnits(0,Amps);
            appliedVoltage = Voltage.ofBaseUnits(0, Volts);
            theta = Angle.ofBaseUnits(0,Radians);
            omega = AngularVelocity.ofBaseUnits(0, RadiansPerSecond);
        }
    }

    public static class ModuleSignals
    {
        public StatusSignal<Angle> positionSignal;
        public StatusSignal<AngularVelocity> velocitySignal;
        public StatusSignal<AngularAcceleration> accelerationSignal;
        public StatusSignal<Current> currentSignal;
        public StatusSignal<Voltage> appliedVoltageSignal;
        public StatusSignal<Angle> thetaSignal;
        public StatusSignal<AngularVelocity> omegaSignal;

        public ModuleSignals(StatusSignal<Angle> positionSignal, StatusSignal<AngularVelocity> velocitySignal, StatusSignal<AngularAcceleration> accelerationSignal, StatusSignal<Current> currentSignal, StatusSignal<Voltage> appliedVoltageSignal, StatusSignal<Angle> thetaSignal, StatusSignal<AngularVelocity> omegaSignal) {
            this.positionSignal = positionSignal;
            this.velocitySignal = velocitySignal;
            this.accelerationSignal = accelerationSignal;
            this.currentSignal = currentSignal;
            this.appliedVoltageSignal = appliedVoltageSignal;
            this.thetaSignal = thetaSignal;
            this.omegaSignal = omegaSignal;
        }

        public ModuleSignals() {}
    }

    public Observer(ModuleSignals frontLeft, ModuleSignals frontRight, ModuleSignals backLeft, ModuleSignals backRight, StatusSignal<Angle> yaw) {
        CANBus drivetrainBus = new CANBus(DrivetrainConstants.CAN_BUS);
        while (!drivetrainBus.getStatus().Status.isOK());

        observationsLock = new ReentrantReadWriteLock();
        observations = new ArrayList<>();
        observationsLock.writeLock().lock();
        observations.add(new SwerveObservation(new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new Rotation2d(0),0));
        observationsLock.writeLock().unlock();

        moduleObservationLock = new ReentrantReadWriteLock();

        frontLeftObservation = new ModuleObservationRaw();
        frontLeftSignals = frontLeft;

        frontRightObservation = new ModuleObservationRaw();
        frontRightSignals = frontRight;

        backLeftObservation = new ModuleObservationRaw();
        backLeftSignals = backLeft;

        backRightObservation = new ModuleObservationRaw();
        backRightSignals = backRight;
        
        yawLock = new ReentrantReadWriteLock();
        yawSignal = yaw;
        // The can bus is at 50% usage at 500 hz, should be able to go up to 800
        BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                frontLeftSignals.positionSignal,
                frontLeftSignals.velocitySignal,
                frontLeftSignals.accelerationSignal,
                frontLeftSignals.currentSignal,
                frontLeftSignals.appliedVoltageSignal,
                frontLeftSignals.thetaSignal,
                frontLeftSignals.omegaSignal,

                frontRightSignals.positionSignal,
                frontRightSignals.velocitySignal,
                frontRightSignals.accelerationSignal,
                frontRightSignals.currentSignal,
                frontRightSignals.appliedVoltageSignal,
                frontRightSignals.thetaSignal,
                frontRightSignals.omegaSignal,

                backLeftSignals.positionSignal,
                backLeftSignals.velocitySignal,
                backLeftSignals.accelerationSignal,
                backLeftSignals.currentSignal,
                backLeftSignals.appliedVoltageSignal,
                backLeftSignals.thetaSignal,
                backLeftSignals.omegaSignal,

                backRightSignals.positionSignal,
                backRightSignals.velocitySignal,
                backRightSignals.accelerationSignal,
                backRightSignals.currentSignal,
                backRightSignals.thetaSignal,
                backRightSignals.omegaSignal,

                yawSignal
        );
    }

    @Override
    public void run() {
        while (true) {
            BaseStatusSignal.waitForAll(
                    2,
                    frontLeftSignals.positionSignal,
                    frontLeftSignals.velocitySignal,
                    frontLeftSignals.accelerationSignal,
                    frontLeftSignals.currentSignal,
                    frontLeftSignals.appliedVoltageSignal,
                    frontLeftSignals.thetaSignal,
                    frontLeftSignals.omegaSignal,

                    frontRightSignals.positionSignal,
                    frontRightSignals.velocitySignal,
                    frontRightSignals.accelerationSignal,
                    frontRightSignals.currentSignal,
                    frontRightSignals.appliedVoltageSignal,
                    frontRightSignals.thetaSignal,
                    frontRightSignals.omegaSignal,

                    backLeftSignals.positionSignal,
                    backLeftSignals.velocitySignal,
                    backLeftSignals.accelerationSignal,
                    backLeftSignals.currentSignal,
                    backLeftSignals.appliedVoltageSignal,
                    backLeftSignals.thetaSignal,
                    backLeftSignals.omegaSignal,

                    backRightSignals.positionSignal,
                    backRightSignals.velocitySignal,
                    backRightSignals.accelerationSignal,
                    backRightSignals.currentSignal,
                    backRightSignals.thetaSignal,
                    backRightSignals.omegaSignal,

                    yawSignal
            );
            moduleObservationLock.writeLock().lock();

            frontLeftObservation = SignalExtract(frontLeftSignals);
            frontRightObservation = SignalExtract(frontRightSignals);
            backLeftObservation = SignalExtract(backLeftSignals);
            backRightObservation = SignalExtract(backRightSignals);

            moduleObservationLock.writeLock().unlock();

            observationsLock.writeLock().lock();
            moduleObservationLock.readLock().lock();
            yawLock.readLock().lock();

            observations.add(
                    new SwerveObservation(
                            new SwerveModulePosition((frontLeftObservation.position.in(Rotations)/DrivetrainConstants.DRIVE_GEAR_RATIO) * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI, Rotation2d.fromRotations(frontLeftObservation.theta.in(Rotations))),
                            new SwerveModulePosition((frontRightObservation.position.in(Rotations)/DrivetrainConstants.DRIVE_GEAR_RATIO) * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI, Rotation2d.fromRotations(frontRightObservation.theta.in(Rotations))),
                            new SwerveModulePosition((backLeftObservation.position.in(Rotations)/DrivetrainConstants.DRIVE_GEAR_RATIO) * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI, Rotation2d.fromRotations(backLeftObservation.theta.in(Rotations))),
                            new SwerveModulePosition((backRightObservation.position.in(Rotations)/DrivetrainConstants.DRIVE_GEAR_RATIO) * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI, Rotation2d.fromRotations(backRightObservation.theta.in(Rotations))),
                            new Rotation2d(getYaw().in(Radians)),
                            Timer.getFPGATimestamp()
                    )
            );

            observationsLock.writeLock().unlock();
            moduleObservationLock.readLock().unlock();
            yawLock.readLock().unlock();

//            try {
//                Thread.sleep(2);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
        }
    }

    public ModuleObservationRaw SignalExtract(ModuleSignals signals) {
        ModuleObservationRaw raw = new ModuleObservationRaw();
        raw.position = StatusSignal.getLatencyCompensatedValue(signals.positionSignal,signals.velocitySignal).copy();
        raw.velocity = StatusSignal.getLatencyCompensatedValue(signals.velocitySignal,signals.accelerationSignal).copy();
        raw.acceleration = signals.accelerationSignal.getValue().copy();
        raw.current = signals.currentSignal.getValue().copy();
        raw.appliedVoltage = signals.appliedVoltageSignal.getValue().copy();
        raw.theta = StatusSignal.getLatencyCompensatedValue(signals.thetaSignal, signals.omegaSignal).copy();
        raw.omega = signals.omegaSignal.getValue().copy();
        return raw;
    }

    public ArrayList<SwerveObservation> getObservations() {
        observationsLock.readLock().lock();
        ArrayList<SwerveObservation> data = observations;
        observationsLock.readLock().unlock();

        return data;
    }

    public void clearObservations() {
        observationsLock.writeLock().lock();
        observations.clear();
        observationsLock.writeLock().unlock();
    }


    public ModuleObservationRaw[] getModuleObservations() {
        moduleObservationLock.readLock().lock();
        var moduleObservations =  new ModuleObservationRaw[]{frontLeftObservation, frontRightObservation, backLeftObservation, backRightObservation};
        moduleObservationLock.readLock().unlock();
        return moduleObservations;
    }

    public Measure<AngleUnit> getYaw() {
        yawLock.readLock().lock();
        Measure<AngleUnit> measure = yawSignal.getValue();
        yawLock.readLock().unlock();
        return measure;
    }
}
