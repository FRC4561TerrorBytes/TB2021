/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.VisionData;
import frc.robot.subsystems.ShooterSubsystem;

public class TurretSetpointCommand extends CommandBase {
  
  ShooterSubsystem subsystem;
  int setpoint;
  boolean visionInterrupt = false;

  /**
   * Creates a new TurretSetpointCommand.
   */
  public TurretSetpointCommand(ShooterSubsystem subsystem, int setpoint, boolean visionInterrupt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.setpoint = setpoint;
    this.visionInterrupt = visionInterrupt;

    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.moveTurretPID(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pass, no need to constantly update a detected variable when it automatically updates anyway
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (this.visionInterrupt && VisionData.isDetected()) subsystem.turretVisionPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subsystem.turretCheckIfMotionComplete() || (this.visionInterrupt ? VisionData.isDetected() : false);
  }
}
