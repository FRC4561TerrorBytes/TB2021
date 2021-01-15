/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.automodes;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoModeConstants;
import frc.robot.AutoModePaths;
import frc.robot.AutoTrajectory;
import frc.robot.commands.AutoMotorsReversed;
import frc.robot.subsystems.DriveSubsystem;

//A Test Auto Mode using PathWeaver Paths and While Going in Reverse.
public class TestReversePathWeaverAuto extends SequentialCommandGroup{
 
  public TestReversePathWeaverAuto(DriveSubsystem driveSubsystem) {

    super(
      //Drives Straight Forward
      new AutoTrajectory(driveSubsystem, AutoModeConstants.DriveStraightTest.trajectoryJSON).getCommand(),
      //Waits 1 second
      new WaitCommand(1),
      //Sets motor rotation to counterclockwise (reverse)
      new AutoMotorsReversed(true),
      //Follows the same path as before, but with motor rotation reversed, so should go in reverse straight backwards
      new AutoTrajectory(driveSubsystem, AutoModePaths.ShootDriveStraight, false).getCommand()
    );
}
}
