/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  ShooterSubsystem shooterSubsystem;
  MagazineSubsystem magazineSubsystem;
  double speed;

  /**
   * Creates a new ShootCommand.
   */
  public ShootCommand(ShooterSubsystem shooterSubsystem, MagazineSubsystem magazineSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.speed = speed;

    addRequirements(this.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooterSubsystem.setFlywheelSpeed(this.speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.shooterSubsystem.isFlywheelAtSpeed()) {
      this.magazineSubsystem.ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
      System.out.println("RUNNING MAGAZINE");
    }

    else {
      this.magazineSubsystem.ballUptakeStop();
      System.out.println("STOPPING...");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.flywheelStop();
    this.magazineSubsystem.ballUptakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
