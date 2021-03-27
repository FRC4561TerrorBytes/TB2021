/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.automodes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTrajectory;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathARedAuto extends SequentialCommandGroup {
  /**
   * Creates a new ShootDriveStraightAuto.
   */
  public PathARedAuto(DriveSubsystem driveSubsystem, MagazineSubsystem magazineSubsystem) {
    super(
      // new AutoTrajectory(driveSubsystem, AutoModePaths.PathA_RedOne, false).getCommand(),
      // new AutoTrajectory(driveSubsystem, AutoModePaths.PathA_RedTwo, false).getCommand(),
      // new AutoTrajectory(driveSubsystem, AutoModePaths.PathA_RedThree, false).getCommand(),
      // new AutoTrajectory(driveSubsystem, AutoModePaths.PathA_RedEnd, false).getCommand()
      new InstantCommand(()-> magazineSubsystem.armSetPosition(Constants.ARM_BOTTOM_POSITION), magazineSubsystem),
      new RunCommand(()-> magazineSubsystem.intakeMotorSpeed(Constants.INTAKE_MOTOR_SPEED), magazineSubsystem).withTimeout(4).alongWith(
      new AutoTrajectory(driveSubsystem, "output/PathA_RedOne.wpilib.json").getCommand()),
      new AutoTrajectory(driveSubsystem, "output/PathA_RedTwo.wpilib.json").getCommand(),
      new AutoTrajectory(driveSubsystem, "output/PathA_RedThree.wpilib.json").getCommand(),
      new InstantCommand(()-> magazineSubsystem.intakeMotorSpeed(Constants.MOTOR_STOP), magazineSubsystem),
      new AutoTrajectory(driveSubsystem, "output/PathA_RedFour.wpilib.json").getCommand()
    );
  }
}
