// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automodes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTrajectory;
import frc.robot.commands.AutoTurnCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BounceAuto extends SequentialCommandGroup {
  /** Creates a new BounceAuto. */
  public BounceAuto(DriveSubsystem driveSubsystem) {
    // Max Velocity: 2.5
    // Max Acceleration: 0.75

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new AutoTrajectory(driveSubsystem, "output/BounceOne.wpilib.json").getCommand(),
      new AutoTurnCommand(driveSubsystem, 0.125, 170, AutoTurnCommand.Direction.Right),
      new AutoTrajectory(driveSubsystem, "output/BounceTwo.wpilib.json").getCommand(),
      new AutoTurnCommand(driveSubsystem, 0.125, 165, AutoTurnCommand.Direction.Right),
      new AutoTrajectory(driveSubsystem, "output/BounceThree.wpilib.json").getCommand(),
      new AutoTurnCommand(driveSubsystem, 0.125, 165, AutoTurnCommand.Direction.Right),
      new AutoTrajectory(driveSubsystem, "output/BounceFour.wpilib.json").getCommand()
      // new AutoTrajectory(driveSubsystem, AutoModePaths.BounceOne, false).getCommand(),
      // new AutoTurnCommand(driveSubsystem, 180),
      // new AutoTrajectory(driveSubsystem, AutoModePaths.BounceTwo, false).getCommand(),
      // new AutoTurnCommand(driveSubsystem, 180),
      // new AutoTrajectory(driveSubsystem, AutoModePaths.BounceThree, false).getCommand(),
      // new AutoTurnCommand(driveSubsystem, 180),
      // new AutoTrajectory(driveSubsystem, AutoModePaths.BounceFour, false).getCommand()
);
  }
}
