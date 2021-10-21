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
import frc.robot.VisionData;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretSetpointCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShowOffAuto extends SequentialCommandGroup {
  DriveSubsystem driveSubsystem;
  ShooterSubsystem shooterSubsystem;
  MagazineSubsystem magazineSubsystem;
  /**
   * Creates a new TrenchSixBallAuto.
   */
  public ShowOffAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, MagazineSubsystem magazineSubsystem) {

    super(
      new InstantCommand(() -> VisionData.setEnabled(true)),
      new TurretSetpointCommand(shooterSubsystem, Constants.TURRET_BACK_POSITION, true),
      new InstantCommand(() -> shooterSubsystem.toggleHoodPosition(), shooterSubsystem),
      new ShootCommand(shooterSubsystem, 5400).withTimeout(12),
      new InstantCommand(() -> VisionData.setEnabled(false)),
      new AutoTrajectory(driveSubsystem, AutoModePaths.ShowOffOne, false).getCommand(),
      new RunCommand(() -> magazineSubsystem.intakeMotorSpeed(Constants.INTAKE_MOTOR_SPEED), magazineSubsystem).withTimeout(7).alongWith(
        new AutoTrajectory(driveSubsystem, AutoModePaths.ShowOffTwo, false).getCommand()
      ),
      new InstantCommand(() -> magazineSubsystem.intakeMotorSpeed(Constants.MOTOR_STOP), magazineSubsystem),
      new AutoTrajectory(driveSubsystem, AutoModePaths.ShowOffThree, true).getCommand(),
      new InstantCommand(() -> VisionData.setEnabled(true)),
      new InstantCommand(()-> shooterSubsystem.relativeMoveTurretPID(600)),
      new ShootCommand(shooterSubsystem, 5400).withTimeout(12),
      new InstantCommand(() -> VisionData.setEnabled(false)),
      new InstantCommand(()-> shooterSubsystem.toggleHoodPosition(), shooterSubsystem),
      new TurretSetpointCommand(shooterSubsystem, Constants.TURRET_FRONT_LIMIT_POSITION, false)
    );
  }
}
