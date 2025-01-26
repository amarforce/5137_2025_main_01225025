package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ArmConstants;

public class DropCoralOnL4Command extends SequentialCommandGroup {

    public DropCoralOnL4Command(Elevator elevator, Arm arm, Vision vision, Swerve drive) {
        addCommands(
            // Step 1: Align the robot to the reef using vision and swerve drive
            new AlignRobotToReefCommand(vision, drive) {
                @Override
                public void execute() {
                    System.out.println("Executing AlignRobotToReefCommand...");
                    if (vision.getFrontPoseEstimate().isEmpty()) {
                        System.err.println("Vision data unavailable. Alignment cannot proceed.");
                        cancel();
                    } else {
                        System.out.println("Vision alignment data received.");
                        super.execute();
                    }
                }

                @Override
                public boolean isFinished() {
                    boolean dataAvailable = vision.getFrontPoseEstimate().isPresent();
                    System.out.println("Vision data available: " + dataAvailable);
                    return dataAvailable;
                }
            },

            // Step 2: Raise the elevator to the L4 height
            new MoveElevatorToL4Command(elevator) {
                @Override
                public void initialize() {
                    System.out.println("Initializing MoveElevatorToL4Command...");
                    // Ensure the elevator limit is checked thoroughly under all conditions
                    if (elevator.isAtLimit()) {
                        System.err.println("Elevator limit reached. Movement stopped to protect hardware.");
                        cancel();
                    } else {
                        System.out.println("Setting elevator goal to L4 height.");
                        elevator.setGoal(ElevatorConstants.L4goal);
                    }
                }

                @Override
                public boolean isFinished() {
                    boolean atSetpoint = elevator.atSetpoint();
                    System.out.println("Elevator at setpoint: " + atSetpoint);
                    return atSetpoint;
                }

                @Override
                public void end(boolean interrupted) {
                    if (interrupted) {
                        System.out.println("Elevator Movement Interrupted");
                    } else {
                        System.out.println("Elevator Reached L4");
                    }
                }
            },

            // Step 3: Position the arm for L4 coral placement
            new PositionArmForL4PlacementCommand(arm) {
                @Override
                public void initialize() {
                    System.out.println("Initializing PositionArmForL4PlacementCommand...");
                    arm.setGoal(ArmConstants.L4Angle);
                }

                @Override
                public boolean isFinished() {
                    boolean withinTolerance = Math.abs(arm.getPose() - ArmConstants.L4Angle) <= ArmConstants.tolerance;
                    System.out.println("Arm within tolerance: " + withinTolerance);
                    return withinTolerance;
                }

                @Override
                public void execute() {
                    super.execute();
                    System.out.println("Executing PositionArmForL4PlacementCommand...");
                    long timeout = 5000;
                    if (System.currentTimeMillis() - this.timeSinceInitialized() > timeout) {
                        // Log timeout details to aid debugging
                        System.err.println("Arm positioning timeout after " + timeout + "ms. Current pose: " + arm.getPose());
                        cancel();
                    }
                }

                @Override
                public void end(boolean interrupted) {
                    if (interrupted) {
                        System.out.println("Arm Movement Interrupted");
                    } else {
                        System.out.println("Arm Positioned for L4 Placement");
                    }
                }
            },

            // Step 4: Release the coral
            new ReleaseCoralCommand(arm) {
                @Override
                public void initialize() {
                    System.out.println("Initializing ReleaseCoralCommand...");
                    arm.setSpeed(0.0);
                }

                @Override
                public boolean isFinished() {
                    boolean success = arm.isCoralReleased();
                    if (!success) {
                        System.err.println("Coral release failed. Check mechanism.");
                    } else {
                        System.out.println("Coral successfully released.");
                    }
                    return success;
                }

                @Override
                public void end(boolean interrupted) {
                    if (interrupted) {
                        System.err.println("Coral Release Interrupted");
                    } else {
                        System.out.println("Coral Release Complete");
                    }
                }
            },

            // Step 5: Reset the elevator and arm to their default positions
            new ResetSubsystemsCommand(elevator, arm) {
                @Override
                public void initialize() {
                    System.out.println("Initializing ResetSubsystemsCommand...");
                    elevator.setGoal(ElevatorConstants.defaultHeight);
                    arm.setGoal(ArmConstants.defaultAngle);
                }

                @Override
                public void execute() {
                    System.out.println("Executing ResetSubsystemsCommand...");
                    // Use a non-blocking delay to avoid impacting responsiveness
                    if (!super.isFinished()) {
                        System.out.println("Resetting subsystems in progress...");
                    }
                }

                @Override
                public boolean isFinished() {
                    boolean finished = elevator.atSetpoint() && Math.abs(arm.getPose() - ArmConstants.defaultAngle) <= ArmConstants.tolerance;
                    System.out.println("ResetSubsystemsCommand finished: " + finished);
                    return finished;
                }

                @Override
                public void end(boolean interrupted) {
                    if (interrupted) {
                        System.err.println("Subsystem Reset Interrupted");
                    } else {
                        System.out.println("Subsystems Reset Complete");
                    }
                }
            }
        );
    }
}
