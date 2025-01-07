package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;

public interface SwerveModuleIO {

    /**
     * @param targetState Sets the target state for the module this should be called periodically. This state is applied in the writePeriodic function so this must be called before that if you want it to be applied
     */
    void setTargetState(SwerveModuleState targetState);

    void readPeriodic(Observer.ModuleObservationRaw observation);
    /**
     * This should be called periodically after the swerve module state is set
     */
    void writePeriodic();

    /**
     * This should be called in the drivetrain subsystem sim periodic function
     */
    void moduleSim();


    /**
     * @return The value was last updated during the read periodic, if you need at a specific timestamp for odometry use the overload that takes in a timestamp
     */
    @Logged (name = "Wheel Position")
    Distance getPosition();


    /**
     * @return The value was last updated during the read periodic, if you need at a specific timestamp for odometry use the overload that takes in a timestamp
     */
    @Logged (name = "Wheel Velocity")
    LinearVelocity getVelocity();

    /**
     * @return The value was last updated during the read periodic, if you need at a specific timestamp for odometry use the overload that takes in a timestamp
     */
    @Logged (name = "Wheel Acceleration")
    LinearAcceleration getAcceleration();


    /**
     * @return returns the voltage applied to the drive motor.
     */
    @Logged (name = "Drive Voltage")
    Voltage getDriveAppliedVoltage();

    /**
     * @return This is the current angle the wheel is facing in radians wrapped from -PI to PI
     */
    @Logged (name = "Wheel Angle")
    Angle getTheta();

    /**
     * @return This is the current angular velocity of the wheel
     */
    @Logged (name = "Wheel Omega")
    AngularVelocity getOmega();

    /**
     * @return the horizontal force created by the module
     */
    @Logged (name = "Force")
    Force getForce();

    /**
     * @return Returns of the spinning wheel
     */
    @Logged (name = "Torque")
    Torque getTorque();

    /**
     * @return returns the voltage applied to the steer motor.
     */
    @Logged (name = "Steer Voltage")
    Voltage getSteerAppliedVoltage();

    @Logged (name = "Current State")
    SwerveModuleState getCurrentState();

    public Observer.ModuleSignals getModuleSignals();
}
