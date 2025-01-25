package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Hang_Constants;

public class Hang extends SubsystemBase {
    private final Solenoid clampSolenoid;
    private final Solenoid climbSolenoid;
    private final Compressor compressor;

    public Hang() {
        clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Hang_Constants.Pneumatics.SOLENOID_1_CHANNEL);
        climbSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Hang_Constants.Pneumatics.SOLENOID_2_CHANNEL);
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableAnalog(Hang_Constants.Pneumatics.MIN_PRESSURE, 
                              Hang_Constants.Pneumatics.MAX_PRESSURE);
    }

    public void clampActivate() {
        clampSolenoid.set(true);
    }

    public void clampDeactivate() {
        clampSolenoid.set(false);
    }

    public boolean isClampActivated() {
        return clampSolenoid.get();
    }

    public void climbExtend() {
        climbSolenoid.set(true);
    }

    public void climbRetract() {
        climbSolenoid.set(false);
    }

    public boolean isClimbExtended() {
        return climbSolenoid.get();
    }

    @Override
    public void periodic() {
        // Update dashboard with hang system status
        SmartDashboard.putBoolean("Hang/Clamp Active", isClampActivated());
        SmartDashboard.putBoolean("Hang/Climb Extended", isClimbExtended());
        SmartDashboard.putNumber("Hang/Pressure", compressor.getPressure());
    }
}