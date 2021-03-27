// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathABlueAuto extends SequentialCommandGroup {
  /** Creates a new PathABlueAuto. */
  public PathABlueAuto(DriveSubsystem driveSubsystem, MagazineSubsystem magazineSubsystem) {
  super(
    new InstantCommand(()-> magazineSubsystem.armSetPosition(Constants.ARM_BOTTOM_POSITION), magazineSubsystem),
    new RunCommand(()-> magazineSubsystem.intakeMotorSpeed(Constants.INTAKE_MOTOR_SPEED), magazineSubsystem).withTimeout(4).alongWith(
    new AutoTrajectory(driveSubsystem, "output/PathA_BlueOne.wpilib.json").getCommand()).andThen(
    new AutoTrajectory(driveSubsystem, "output/PathA_BlueTwo.wpilib.json").getCommand()).andThen(
    new AutoTrajectory(driveSubsystem, "output/PathA_BlueThree.wpilib.json").getCommand()).andThen(
    new AutoTrajectory(driveSubsystem, "output/PathA_BlueFour.wpilib.json").getCommand()),
    new InstantCommand(()-> magazineSubsystem.intakeMotorSpeed(Constants.MOTOR_STOP), magazineSubsystem)
);
  }
}
