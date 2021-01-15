package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.VisionData;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A command to drive the robot to an optimal distance from the target
 * @author Zane
 */
public class AutoDriveCommand extends CommandBase {
  private final DriveSubsystem DRIVE_SUBSYSTEM;
  private final ShooterSubsystem SHOOTER_SUBSYSTEM;

  public AutoDriveCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
    DRIVE_SUBSYSTEM = drive;
    SHOOTER_SUBSYSTEM = shooter;
    addRequirements(drive, shooter);
  }

  @Override
  public void execute() {
    // There are two types of robots:
    // Those who can auto drive from incomplete data...
    if (!VisionData.isReady()) return;
    // Also don't drive towards a target you can't see
    if (!VisionData.isDetected()) return;

    // Find angle of robot relative to target by using shooter camera as reference
    double robotToTargetAngle = SHOOTER_SUBSYSTEM.getTurretAngle() + VisionData.getXAngle();
    // Damper required to not burn the carpet again
    // Turns the robot towards xAngle
    DRIVE_SUBSYSTEM.setSetpoint(
        DRIVE_SUBSYSTEM.getController().getSetpoint() + Constants.TURN_DAMPER * robotToTargetAngle);

    // If target is detected, move to a pre-defined distance
    // Checks if angle is within a certain value to prevent veering too far off course
    if (VisionData.isReady() && robotToTargetAngle <= Constants.ANGLE_TOLERANCE) {
      // Find distance of robot from target in terms of stopping distance and normalize it
      // TODO: Experiment with different easing functions to increase accuracy
      double stoppingDistancePer = (VisionData.getDistance() - Constants.TARGET_DISTANCE) / Constants.AUTO_STOPPING_DISTANCE;
      stoppingDistancePer = Math.max(Math.min(stoppingDistancePer, 1.0), -1.0);
      // If robot has x% of stopping distance left, move at x% speed
      DRIVE_SUBSYSTEM.teleopPID(Constants.AUTO_SPEED * stoppingDistancePer, 0.0, 1);
    }
  }

}