/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.automodes;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoModePaths;
import frc.robot.AutoTrajectory;
import frc.robot.Constants;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretSetpointCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootDriveStraightAuto extends SequentialCommandGroup {
  /**
   * Creates a new ShootDriveStraightAuto.
   */
  public ShootDriveStraightAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, MagazineSubsystem magazineSubsystem) {

    super(
      new TurretSetpointCommand(shooterSubsystem, Constants.TURRET_BACK_POSITION, false),
      new InstantCommand(()-> shooterSubsystem.toggleHoodPosition(), shooterSubsystem),
      new ShootCommand(shooterSubsystem, magazineSubsystem, 5400).withTimeout(8),
      new WaitCommand(1),
      new AutoTrajectory(driveSubsystem, AutoModePaths.ShootDriveStraight, false).getCommand(),
      new InstantCommand(()-> shooterSubsystem.toggleHoodPosition(), shooterSubsystem),
      new TurretSetpointCommand(shooterSubsystem, Constants.TURRET_FRONT_LIMIT_POSITION, false)
    );
  }
}
