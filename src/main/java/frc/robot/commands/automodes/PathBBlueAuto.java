// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automodes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTrajectory;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathBBlueAuto extends SequentialCommandGroup {
  /** Creates a new PathBBlueAuto. */
  public PathBBlueAuto(DriveSubsystem driveSubsystem) {
  super(
    new AutoTrajectory(driveSubsystem, "output/PathB_BlueOne.wpilib.json").getCommand(),
  new AutoTrajectory(driveSubsystem, "output/PathB_BlueTwo.wpilib.json").getCommand(),
  new AutoTrajectory(driveSubsystem, "output/PathB_BlueThree.wpilib.json").getCommand(),
  new AutoTrajectory(driveSubsystem, "output/PathB_BlueFour.wpilib.json").getCommand()
  );
  }
}
