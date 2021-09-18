// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.MagazineSubsystem;

public class UptakeCommand extends CommandBase {
MagazineSubsystem subsystem;

  /** Creates a new UptakeCommand. */
  public UptakeCommand(MagazineSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subsystem.ballUptake(0.5);
    try {Thread.sleep(50);} catch (Exception e) {}
    this.subsystem.ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(50);} catch (Exception e) {}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
