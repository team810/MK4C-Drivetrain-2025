package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.lib.AdvancedSubsystem;
import frc.robot.lib.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends AdvancedSubsystem {
    private static ClimberSubsystem instance;

    private final DoubleSolenoid legPistons;
    private final DoubleSolenoid clampPistons;

    private double tx;
    private double ty;

    public ClimberSubsystem() {
        legPistons = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                ClimberConstants.FWD_CHANNEL_LEG,
                ClimberConstants.REV_CHANNEL_LEG
        );
        clampPistons = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                ClimberConstants.FWD_CHANNEL_CLAMP,
                ClimberConstants.REV_CHANNEL_CLAMP
        );
        legPistons.set(DoubleSolenoid.Value.kOff);
        clampPistons.set(DoubleSolenoid.Value.kOff);

        tx = 0;
        ty = 0;

        if (ClimberConstants.USE_CLIMB_LL) {
            tx = LimelightHelpers.getTX(ClimberConstants.CLIMB_LL);
            ty = LimelightHelpers.getTY(ClimberConstants.CLIMB_LL);
        }
    }

    @Override
    public void readPeriodic() {
        if (ClimberConstants.USE_CLIMB_LL) {
            tx = LimelightHelpers.getTX(ClimberConstants.CLIMB_LL);
            ty = LimelightHelpers.getTY(ClimberConstants.CLIMB_LL);
        }

        Logger.recordOutput("ClimberSubsystem/LegPistonState", legPistons.get());
        Logger.recordOutput("ClimberSubsystem/ClampState", clampPistons.get());
        Logger.recordOutput("ClimberSubsystem/Tx", tx);
        Logger.recordOutput("ClimberSubsystem/TY", ty);

    }

    @Override
    public void writePeriodic() {
        // Add for simulation stuff
    }

    @Override
    public void simulatePeriodic() {
        // Add for simulation stuff
    }

    public void setLegPistons(DoubleSolenoid.Value value) {
        legPistons.set(value);
    }

    public void setClampPistons(DoubleSolenoid.Value value) {
        clampPistons.set(value);
    }

    public boolean usingLL() {
        return ClimberConstants.USE_CLIMB_LL;
    }

    public double getTx() {
        return tx;
    }
    public double getTY() {
        return ty;
    }

    public void changeAlliance(DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Blue) {
            LimelightHelpers.setPipelineIndex(
                    ClimberConstants.CLIMB_LL,
                    ClimberConstants.BLUE_PIPELINE
            );
        }else{
            LimelightHelpers.setPipelineIndex(
                    ClimberConstants.CLIMB_LL,
                    ClimberConstants.RED_PIPELINE
            );
        }
    }

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }
        return instance;
    }
}
