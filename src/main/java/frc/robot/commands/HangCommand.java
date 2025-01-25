package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Hang;

/**
 * Command sequence for the hanging operation
 * Executes a series of steps:
 * 1. Deactivate clamp
 * 2. Extend climbing mechanism
 * 3. Activate clamp
 * 4. Retract climbing mechanism
 */
public class HangCommand extends SequentialCommandGroup {
    private static final double STEP_TIMEOUT = 2.0; // Timeout for each step
    
    public HangCommand(Hang hangSubsystem) {
        addCommands(
            // Step 1: Release clamp
            Commands.runOnce(() -> hangSubsystem.clampDeactivate())
                .withTimeout(STEP_TIMEOUT),
            
            new WaitCommand(1.0),
            
            // Step 2: Extend climbing mechanism
            Commands.runOnce(() -> hangSubsystem.climbExtend())
                .withTimeout(STEP_TIMEOUT),
            
            new WaitCommand(1.0),
            
            // Step 3: Activate clamp
            Commands.runOnce(() -> hangSubsystem.clampActivate())
                .withTimeout(STEP_TIMEOUT),
            
            new WaitCommand(1.0),
            
            // Step 4: Retract climbing mechanism
            Commands.runOnce(() -> hangSubsystem.climbRetract())
                .withTimeout(STEP_TIMEOUT)
        );
    }
}
