// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurnCommand extends CommandBase {
  DriveSubsystem subsystem;
  double angle;
  double leftSpeed, rightSpeed;
  public static enum Direction {
    Left, Right;
  }
  Direction direction;


  /** Creates a new AutoTurnCommand. */
  public AutoTurnCommand(DriveSubsystem subsystem, double speed, double angle, Direction direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.angle = angle;
    this.direction = direction;
    this.leftSpeed = speed;
    this.rightSpeed = speed;

    if (this.direction == Direction.Left) {
      rightSpeed *= -1;
    } else {
      leftSpeed *= -1;
    }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.resetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.tankDrive(leftSpeed, rightSpeed);
    System.out.println("Turning lmao " + leftSpeed + " " + rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(subsystem.getAngle() - angle) <= 3;
  }
}
