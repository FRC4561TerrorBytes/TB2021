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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class TrenchSixBallAuto extends SequentialCommandGroup {
  DriveSubsystem driveSubsystem;
  ShooterSubsystem shooterSubsystem;
  MagazineSubsystem magazineSubsystem;
  /**
   * Creates a new TrenchSixBallAuto.
   */
  public TrenchSixBallAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, MagazineSubsystem magazineSubsystem) {

    super(
      new TurretSetpointCommand(shooterSubsystem, Constants.TURRET_SIXBALL_POSITION, false),
      new InstantCommand(() -> shooterSubsystem.toggleHoodPosition(), shooterSubsystem),
      new ShootCommand(shooterSubsystem, magazineSubsystem, 5400).withTimeout(3),
      new RunCommand(() -> magazineSubsystem.intakeMotorSpeed(Constants.INTAKE_MOTOR_SPEED), magazineSubsystem).alongWith(
        new AutoTrajectory(driveSubsystem, AutoModePaths.TrenchSixBallPt1, false).getCommand()
      ).withTimeout(5),
      new WaitCommand(0),
      new RunCommand(() -> magazineSubsystem.intakeMotorSpeed(Constants.MOTOR_STOP), magazineSubsystem).withTimeout(0),
      new AutoTrajectory(driveSubsystem, AutoModePaths.TrenchSixBallPt2, true).getCommand(),
      new TurretSetpointCommand(shooterSubsystem, Constants.TURRET_BACK_LIMIT_POSITION, false),
      new ShootCommand(shooterSubsystem, magazineSubsystem, 5400).withTimeout(3),
      new InstantCommand(()-> shooterSubsystem.toggleHoodPosition(), shooterSubsystem),
      new TurretSetpointCommand(shooterSubsystem, Constants.TURRET_FRONT_LIMIT_POSITION, false)
    );
  }
}
