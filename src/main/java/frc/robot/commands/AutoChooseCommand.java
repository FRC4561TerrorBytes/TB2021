// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.automodes.PathABlueAuto;
import frc.robot.commands.automodes.PathARedAuto;
import frc.robot.commands.automodes.PathBBlueAuto;
import frc.robot.commands.automodes.PathBRedAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class AutoChooseCommand extends CommandBase {
  SequentialCommandGroup commandGroup = null;
  DriveSubsystem driveSubsystem;
  MagazineSubsystem magazineSubsystem;

  /** Creates a new AutoChooseCommand. */
  public AutoChooseCommand(DriveSubsystem driveSubsystem, MagazineSubsystem magazineSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.magazineSubsystem = magazineSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (this.driveSubsystem.getLIDAR() >= 1) {
      if (this.driveSubsystem.getLIDAR() >= 3.7)
        commandGroup = new PathABlueAuto(this.driveSubsystem, this.magazineSubsystem);
      else if (this.driveSubsystem.getLIDAR() >= 2.6)
        commandGroup = new PathBBlueAuto(this.driveSubsystem, this.magazineSubsystem);
      else if (this.driveSubsystem.getLIDAR() >= 2.2)
        commandGroup = new PathARedAuto(this.driveSubsystem, this.magazineSubsystem);
      else if (this.driveSubsystem.getLIDAR() >= 1.1)
        commandGroup = new PathBRedAuto(this.driveSubsystem, this.magazineSubsystem);
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
