package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorTalonFX implements ElevatorIO{
    private final TalonFX primary;
    private final TalonFX secondary;

    public ElevatorTalonFX() {
        primary = new TalonFX(ElevatorConstants.PRIMARY_MOTOR_ID);
        secondary = new TalonFX(ElevatorConstants.SECONDARY_MOTOR_ID);
    }

    @Override
    public void setElevator(double setpoint) {

    }

    @Override
    public boolean atSetpoint() {
        return false;
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {

    }
}
