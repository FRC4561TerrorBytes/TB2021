// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.automodes.PathABlueAuto;
import frc.robot.commands.automodes.PathARedAuto;
import frc.robot.commands.automodes.PathBRedAuto;
import frc.robot.subsystems.DriveSubsystem;

public class AutoChooseCommand extends CommandBase {
  SequentialCommandGroup commandGroup = null;
  DriveSubsystem subsystem;

  /** Creates a new AutoChooseCommand. */
  public AutoChooseCommand(DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (this.subsystem.getLIDAR() >= 1) {
      commandGroup = new PathABlueAuto(this.subsystem);
    } else if (this.subsystem.getLIDAR() >= 1.5) {
      commandGroup = new PathARedAuto(this.subsystem);
    } else if (this.subsystem.getLIDAR() >= 1.5) {
      commandGroup = new PathABlueAuto(this.subsystem);
    } else if (this.subsystem.getLIDAR() >= 2) {
      commandGroup = new PathBRedAuto(this.subsystem);
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandGroup.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
