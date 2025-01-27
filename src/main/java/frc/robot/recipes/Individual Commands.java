package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ArmConstants;

/**
 * Command to align the robot to the reef using vision and swerve drive.
 */
public class AlignRobotToReefCommand extends CommandBase {
    private final Vision vision;
    private final Swerve drive;

    /**
     * Constructs a new AlignRobotToReefCommand.
     *
     * @param vision The vision subsystem used for alignment.
     * @param drive  The swerve drive subsystem used for movement.
     */
    public AlignRobotToReefCommand(Vision vision, Swerve drive) {
        this.vision = vision;
        this.drive = drive;
        addRequirements(vision, drive); // This command requires the vision and drive subsystems
    }
    

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        System.out.println("Aligning to Reef...");
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        System.out.println("Executing AlignRobotToReefCommand...");
        // Cache vision data to avoid redundant calls
        var cachedPose = vision.getFrontPoseEstimate();
        if (cachedPose.isEmpty()) {
            System.err.println("Vision data unavailable. Alignment cannot proceed.");
            cancel(); // Cancel the command if vision data is unavailable
        } else {
            System.out.println("Vision alignment data received.");
            // Use the vision data to drive to the target pose
            cachedPose.ifPresent(pose -> {
                System.out.println("Driving to pose: " + pose.estimatedPose.toPose2d());
                drive.driveToPose(pose.estimatedPose.toPose2d());
            });
        }
    }

    /**
     * Returns true when the command should end.
     *
     * @return True if the robot has reached the target pose.
     */
    @Override
    public boolean isFinished() {
        // Check if the robot has reached the target pose (implementation needed)
        boolean isFinished = drive.atSetpoint(); // Replace with actual check
        return isFinished;
    }

    /**
     * Called once the command ends or is interrupted.
     *
     * @param interrupted True if the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Alignment Interrupted");
        } else {
            System.out.println("Alignment Complete");
        }
    }
}

/**
 * Command to move the elevator to the L4 height.
 */
public class MoveElevatorToL4Command extends CommandBase {
    private final Elevator elevator;

    /**
     * Constructs a new MoveElevatorToL4Command.
     *
     * @param elevator The elevator subsystem.
     */
    public MoveElevatorToL4Command(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator); // This command requires the elevator subsystem
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        System.out.println("Initializing MoveElevatorToL4Command...");
        // Check if the elevator has reached its limit
        if (elevator.isAtLimit()) {
            System.err.println("Elevator limit reached. Checking hardware state...");
            // Verify the limit sensors to ensure they are working correctly
            if (!elevator.verifyLimitSensors()) {
                System.err.println("Limit sensor check failed. Stopping to protect hardware.");
                cancel(); // Cancel the command if the limit sensors are not working
            } else {
                System.err.println("Limit sensors verified. Movement stopped to prevent hardware damage.");
                cancel(); // Cancel the command if the elevator is at its limit
            }
        } else {
            System.out.println("Setting elevator goal to L4 height.");
            // Set the elevator goal to the L4 height
            elevator.setGoal(ElevatorConstants.L4goal);
        }
    }

    /**
     * Returns true when the command should end.
     *
     * @return True if the elevator has reached the L4 setpoint.
     */
    @Override
    public boolean isFinished() {
        boolean atSetpoint = elevator.atSetpoint();
        System.out.println("Elevator at setpoint: " + atSetpoint);
        return atSetpoint;
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        // Verify the limit sensors during operation to prevent hardware damage
        if (!elevator.verifyLimitSensors()) {
            System.err.println("Elevator limit sensor failure detected during operation. Cancelling command to prevent hardware damage.");
            cancel(); // Cancel the command if the limit sensors are not working
        }
    }

    /**
     * Called once the command ends or is interrupted.
     *
     * @param interrupted True if the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Elevator Movement Interrupted");
        } else {
            System.out.println("Elevator Reached L4");
        }
    }
}

/**
 * Command to position the arm for L4 coral placement.
 */
public class PositionArmForL4PlacementCommand extends CommandBase {
    private final Arm arm;

    /**
     * Constructs a new PositionArmForL4PlacementCommand.
     *
     * @param arm The arm subsystem.
     */
    public PositionArmForL4PlacementCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm); // This command requires the arm subsystem
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        System.out.println("Initializing PositionArmForL4PlacementCommand...");
        // Set the arm goal to the L4 angle
        arm.setGoal(ArmConstants.L4Angle);
    }

    /**
     * Returns true when the command should end.
     *
     * @return True if the arm is within tolerance of the L4 angle.
     */
    @Override
    public boolean isFinished() {
        // Check if the arm is within tolerance of the target angle
        boolean withinTolerance = Math.abs(arm.getPose() - ArmConstants.L4Angle) <= ArmConstants.tolerance;
        System.out.println("Arm within tolerance: " + withinTolerance);
        return withinTolerance;
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        super.execute();
        System.out.println("Executing PositionArmForL4PlacementCommand...");
        long timeout = 5000; // Timeout in milliseconds
        // Calculate elapsed time since command initialization
        long elapsedTime = System.currentTimeMillis() - this.timeSinceInitialized();
        System.out.println("Elapsed time: " + elapsedTime + "ms");
        // Check for timeout
        if (elapsedTime > timeout) {
            // Log timeout details to aid debugging
            System.err.println("Arm positioning timeout after " + timeout + "ms. Current pose: " + arm.getPose());
            System.err.println("Resetting arm state to default.");
            // Reset arm to default angle
            arm.setGoal(ArmConstants.defaultAngle);
            cancel(); // Cancel the command if the timeout is reached
        }
    }

    /**
     * Called once the command ends or is interrupted.
     *
     * @param interrupted True if the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Arm Movement Interrupted");
        } else {
            System.out.println("Arm Positioned for L4 Placement");
        }
    }
}

/**
 * Command to release the coral.
 */
public class ReleaseCoralCommand extends CommandBase {
    private final Arm arm;
    private int retryCount = 0;
    private static final int MAX_RETRIES = 3;

    /**
     * Constructs a new ReleaseCoralCommand.
     *
     * @param arm The arm subsystem.
     */
    public ReleaseCoralCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm); // This command requires the arm subsystem
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        System.out.println("Initializing ReleaseCoralCommand...");
        // Stop the arm before releasing the coral
        arm.setSpeed(0.0);
    }

    /**
     * Returns true when the command should end.
     *
     * @return True if the coral has been released successfully.
     */
    @Override
    public boolean isFinished() {
        // Check if the coral has been released
        boolean success = arm.isCoralReleased();
        // Check if the coral has been partially released
        boolean partialRelease = arm.getReleaseSensorData();
        if (!success && retryCount < MAX_RETRIES) {
            // Retry releasing the coral if it has not been released and the maximum retries have not been reached
            System.err.println("Coral release failed. Retrying... (Attempt " + (retryCount + 1) + " of " + MAX_RETRIES + ")");
            retryCount++;
            arm.setSpeed(0.0); // Stop the arm before retrying
            return false; // Continue the command
        } else if (!success) {
            // The coral release failed after maximum retries
            System.err.println("Coral release failed after maximum retries. Check mechanism.");
        } else if (partialRelease) {
            // The coral has been partially released
            System.err.println("Coral partially released. Ensure complete disengagement.");
        } else {
            // The coral has been successfully released
            System.out.println("Coral successfully released.");
        }
        // End the command if the coral has been successfully released and is not partially released
        return success && !partialRelease;
    }

    /**
     * Called once the command ends or is interrupted.
     *
     * @param interrupted True if the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.err.println("Coral Release Interrupted");
        } else {
            System.out.println("Coral Release Complete");
        }
    }
}

/**
 * Command to reset the elevator and arm to their default positions.
 */
public class ResetSubsystemsCommand extends CommandBase {
    private final Elevator elevator;
    private final Arm arm;

    /**
     * Constructs a new ResetSubsystemsCommand.
     *
     * @param elevator The elevator subsystem.
     * @param arm      The arm subsystem.
     */
    public ResetSubsystemsCommand(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        addRequirements(elevator, arm); // This command requires the elevator and arm subsystems
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        System.out.println("Initializing ResetSubsystemsCommand...");
        // Set the elevator and arm goals to their default positions
        elevator.setGoal(ElevatorConstants.defaultHeight);
        arm.setGoal(ArmConstants.defaultAngle);
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        System.out.println("Executing ResetSubsystemsCommand...");
        if (!super.isFinished()) {
            // Check for invalid elevator positions
            double currentElevatorPosition = elevator.getCurrentPosition();
            if (Double.isNaN(currentElevatorPosition) || Double.isInfinite(currentElevatorPosition)) {
                System.err.println("Invalid elevator position detected: " + currentElevatorPosition);
                // Reset the elevator position to a safe state
                elevator.resetPosition();
                cancel(); // Cancel the command if the elevator position is invalid
            } else {
                // Log the elevator and arm positions
                System.out.println("Elevator position: " + currentElevatorPosition);
                System.out.println("Arm position: " + arm.getPose());
            }
        }
    }

    /**
     * Returns true when the command should end.
     *
     * @return True if both the elevator and arm have reached their default positions.
     */
    @Override
    public boolean isFinished() {
        // Check if the elevator and arm have reached their setpoints
        boolean finished = elevator.atSetpoint() && Math.abs(arm.getPose() - ArmConstants.defaultAngle) <= ArmConstants.tolerance;
        System.out.println("ResetSubsystemsCommand finished: " + finished);
        return finished;
    }

    /**
     * Called once the command ends or is interrupted.
     *
     * @param interrupted True if the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.err.println("Subsystem Reset Interrupted");
        } else {
            System.out.println("Subsystems Reset Complete");
        }
    }
}
