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
        if (!isClimbExtended()) { // Safety check
            clampSolenoid.set(true);
            logEvent("Clamp activated");
        } else {
            logError("Cannot activate clamp: Climbing mechanism is currently extended. Please retract it first.");
        }
    }

    public void clampDeactivate() {
        clampSolenoid.set(false);
        logEvent("Clamp deactivated");
    }

    public boolean isClampActivated() {
        return clampSolenoid.get();
    }

    public void climbExtend() {
        if (!isClampActivated()) { // Safety check
            climbSolenoid.set(true);
            logEvent("Climb extended");
        } else {
            logError("Cannot extend climb: Clamp is currently activated. Please deactivate it first.");
        }
    }

    public void climbRetract() {
        climbSolenoid.set(false);
        logEvent("Climb retracted");
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

        // Check for pressure anomalies
        double pressure = compressor.getPressure();
        if (pressure < Hang_Constants.Pneumatics.MIN_PRESSURE || pressure > Hang_Constants.Pneumatics.MAX_PRESSURE) {
            logError("Pressure warning: Current pressure (" + pressure + " PSI) is outside the safe range. Check the system for leaks or malfunctions.");
        }
    }

    private void logEvent(String message) {
        // Log event with timestamp
        System.out.println("[INFO] " + System.currentTimeMillis() + ": " + message);
    }

    private void logError(String message) {
        // Log error with timestamp
        System.err.println("[ERROR] " + System.currentTimeMillis() + ": " + message);
    }
}