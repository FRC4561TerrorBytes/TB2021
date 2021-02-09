/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.automodes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoModePaths;
import frc.robot.AutoTrajectory;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathARedAuto extends SequentialCommandGroup {
  /**
   * Creates a new ShootDriveStraightAuto.
   */
  public PathARedAuto(DriveSubsystem driveSubsystem) {
    super(
      new AutoTrajectory(driveSubsystem, AutoModePaths.PathA_RedOne, false).getCommand(),
      new AutoTrajectory(driveSubsystem, AutoModePaths.PathA_RedTwo, false).getCommand(),
      new AutoTrajectory(driveSubsystem, AutoModePaths.PathA_RedThree, false).getCommand(),
      new AutoTrajectory(driveSubsystem, AutoModePaths.PathA_RedEnd, false).getCommand()
    );
  }
}
