/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


//This is a test class for reverse in Autonomous with Pathweaver Paths
public class AutoMotorsReversed extends CommandBase {
  
DriveSubsystem subsystem;
boolean reversed;
 
  public AutoMotorsReversed(boolean isReversed) {
    // Use addRequirements() here to declare subsystem dependencies.
    reversed = isReversed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(reversed == true) {
      subsystem.ReverseMotors = true;
  }
  else{
     subsystem.ReverseMotors = false;
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //makes sure motors are set to normal if command ends / is interrupted. 
    reversed = false;
    subsystem.ReverseMotors = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //makes sure motors set to normal rotation when command finishes
    reversed = false;
    subsystem.ReverseMotors = false;
    return false;
  }
}
