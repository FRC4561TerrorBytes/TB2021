/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {

  private final String SUBSYSTEM_NAME = "Climber Subsystem";
  
  // Declaration of motors
  private final WPI_TalonSRX CLIMBER_LIFT_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_LIFT_MOTOR_PORT);
  private final WPI_TalonSRX CLIMBER_HOOK_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_HOOK_MOTOR_PORT);
  private final WPI_TalonSRX CLIMBER_BALANCE_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_BALANCE_MOTOR_PORT);


  public ClimberSubsystem() {

    // Display the climber's motor values on a Shuffleboard tab for debugging
    if (Constants.CLIMBER_DEBUG) {
      ShuffleboardTab tab = Shuffleboard.getTab(this.SUBSYSTEM_NAME);
      tab.addNumber("Hook Percent Output", () -> CLIMBER_HOOK_MOTOR.getMotorOutputPercent());
      tab.addNumber("Hook Motor Current", () -> CLIMBER_HOOK_MOTOR.getSupplyCurrent());
      tab.addNumber("Lift Percent Output", () -> CLIMBER_LIFT_MOTOR.getMotorOutputPercent());
      tab.addNumber("Lift Motor Current", () -> CLIMBER_LIFT_MOTOR.getSupplyCurrent());

      tab.addNumber("Balance Motor Current", () -> CLIMBER_BALANCE_MOTOR.getSupplyCurrent());
    }
  }
  
  /**
   * Move climber lift
   * @param liftSpeed speed at which motor lifts at [-1, 1]
   */
  public void liftManual(double liftSpeed) {
    CLIMBER_LIFT_MOTOR.set(liftSpeed);
  }

  /**
   * Moves the climber hook
   * @param hookspeed speed at which the hook moves at [-1, 1]
   */
  public void hookManual(double hookspeed) {
    CLIMBER_HOOK_MOTOR.set(hookspeed);
  }

  /**
   * Moves the climber mouse droid
   * @param speed speed at which the mouse droid moves at [-1, 1]
   */
  public void mouseDroidManual(double speed) {
    CLIMBER_BALANCE_MOTOR.set(speed);
  }

  /**
   * Stops the lift motor
   */
  public void stopLift() {
    CLIMBER_LIFT_MOTOR.set(0);
  }

  /**
   * Stops the balance motor
   */
  public void stopBalance() {
    CLIMBER_BALANCE_MOTOR.set(0);
  }

  /**
   * Stops the hook motor
   */
  public void stopHook() {
    CLIMBER_HOOK_MOTOR.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (Math.abs(RobotContainer.XBOX_CONTROLLER.getY(Hand.kLeft)) > Constants.DEADBAND) {
      CLIMBER_HOOK_MOTOR.set(RobotContainer.XBOX_CONTROLLER.getY(Hand.kLeft));
    }
    if (Math.abs(RobotContainer.XBOX_CONTROLLER.getY(Hand.kRight)) > Constants.DEADBAND) {
      CLIMBER_LIFT_MOTOR.set(RobotContainer.XBOX_CONTROLLER.getY(Hand.kRight));
    
    }
  }
}
