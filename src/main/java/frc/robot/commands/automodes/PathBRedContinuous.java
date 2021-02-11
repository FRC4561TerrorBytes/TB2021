// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automodes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.AutoTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathBRedContinuous extends SequentialCommandGroup {
  /** Creates a new PathBRedContinuous. */
  public PathBRedContinuous(DriveSubsystem driveSubsystem) {
    super(
      new AutoTrajectory(driveSubsystem, "output/PathB_RedAll.wpilib.json").getCommand()
    );
  }
}
