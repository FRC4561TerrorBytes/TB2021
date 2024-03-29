/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MagazineSubsystem;

public class IntakeCommand extends CommandBase {

  MagazineSubsystem subsystem;
  double speed = 0;
  /**
   * Creates a new IntakeCommand.
   */
  public IntakeCommand(MagazineSubsystem subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.intakeMotorSpeed(this.speed);
    subsystem.armManual(0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.intakeMotorSpeed(0);
    subsystem.armManual(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
