/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonPIDConfig;

public class MagazineSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private CANSparkMax intakeMotor;
    private WPI_TalonSRX magazineMotor, armMotor;

    public Hardware(CANSparkMax intakeMotor, WPI_TalonSRX armMotor, WPI_TalonSRX magazineMotor) {
      this.intakeMotor = intakeMotor;
      this.armMotor = armMotor;
      this.magazineMotor = magazineMotor;
    }
  }

  private final String SUBSYSTEM_NAME = "Magazine Subsystem";

  // Declaration of motor
  private CANSparkMax m_intakeMotor;
  private WPI_TalonSRX m_armMotor;
  private WPI_TalonSRX m_magazineMotor;

  private final double TICKS_PER_ROTATION = 4096;
  private final double GEAR_RATIO = 1;
  private final double TICKS_PER_DEGREE = (this.TICKS_PER_DEGREE * this. GEAR_RATIO) / 360;
  private TalonPIDConfig m_config;

  private boolean m_armNeedsReset = true;

  /**
   * Creates a new MagazineSubsystem.
   */
  public MagazineSubsystem(Hardware magazineHardware, TalonPIDConfig config) {
    this.m_intakeMotor = magazineHardware.intakeMotor;
    this.m_armMotor = magazineHardware.armMotor;
    this.m_magazineMotor = magazineHardware.magazineMotor;

    this.m_config = config;

    m_armMotor.configFactoryDefault();
  }

  /**
   * Initialize hardware devices for magazine subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware magazineHardware = new Hardware(new CANSparkMax(Constants.INTAKE_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless),
                                              new WPI_TalonSRX(Constants.ARM_MOTOR_PORT),
                                              new WPI_TalonSRX(Constants.MAGAZINE_MOTOR_PORT));
    return magazineHardware;
  }

  /**
   * Create Shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Arm Motor Output", () -> m_armMotor.getMotorOutputPercent());
    tab.addNumber("Arm Motor Position", () -> m_armMotor.getSelectedSensorPosition());
    tab.addNumber("Arm Motor Setpoint", () -> m_armMotor.getClosedLoopTarget());
    tab.addNumber("Arm Motor Current", () -> m_armMotor.getSupplyCurrent());
    tab.addBoolean("Arm limit switch", () -> m_armMotor.getSensorCollection().isFwdLimitSwitchClosed());
    tab.addNumber("Intake speed", () -> m_intakeMotor.get());
    tab.addNumber("Intake current", () -> m_intakeMotor.getOutputCurrent());
  }
 
  /**
   * Makes the motor for the magazine to spin
   * @param speed (double) how fast you want the motor to spin [-1, 1]
   */
  public void ballUptake(double speed) {
    /*
      Method is called continuously when a button on the xbox controller is held
      will sleep for UPTAKE_MOTOR_DELAY in milliseconds before actually running the uptake to speed
      Allows for maybe smoother intake by doing short, quick intervals of speed
      */
    m_magazineMotor.set(speed);
  }

  /**
   * Stop magazine
   */
  public void ballUptakeStop() {
    m_magazineMotor.set(0);
  }

  /**
   * Move arm manually at specified speed
   * @param speed speed to move arm at [-1, 1]
   */
  public void armManual(double speed) {
    m_armMotor.set(speed);
  }

  /**
   * Moves arm to set position
   * @param setpoint Position where arm goes (ticks)
   */
  public void armSetPosition(double setpoint) {
    // normalise setpoint
    if (setpoint < m_config.getLowerLimit()) setpoint = m_config.getLowerLimit();
    if (setpoint > m_config.getUpperLimit()) setpoint = m_config.getUpperLimit();

    // set feed-forward
    double feedForward = m_config.getkF() * Math.cos(Math.toRadians(encoderPositionToDegrees(m_armMotor.getSelectedSensorPosition())));

    // Move arm toward setpoint
    m_armMotor.set(ControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, feedForward);
  }

  /**
   * Moves arm relative to current setpoint
   * @param setpoint increments to move arm by
   */
  public void armPositionRelative(double setpoint) {
    armSetPosition(m_armMotor.getClosedLoopTarget() + setpoint);
  }

  /**
   * Toggles arm between top and bottom positions
   */
  public void toggleArmPosition() {
    if (m_armMotor.getClosedLoopTarget() == Constants.ARM_TOP_POSITION) {
      armSetPosition(Constants.ARM_BOTTOM_POSITION);
    } else if (m_armMotor.getClosedLoopTarget() == Constants.ARM_BOTTOM_POSITION) {
      armSetPosition(Constants.ARM_TOP_POSITION);
    }
  }

  /**
   * Converts position (ticks) to degrees
   * @param position measured in ticks
   * @return degrees
   */
  public double encoderPositionToDegrees(double position) {
    return (position - Constants.ARM_BOTTOM_POSITION) / TICKS_PER_DEGREE;
  }

  /**
   * Makes the intake motor spin
   * @param speed (double) how fast you want the motor to spin [-1, 1]
   */
  public void intakeMotorSpeed(double speed) {
    m_intakeMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_armMotor.getSensorCollection().isFwdLimitSwitchClosed() && m_armNeedsReset) {
      m_armNeedsReset = false;
      m_armMotor.setSelectedSensorPosition(Constants.ARM_TOP_POSITION);
    } else if (!m_armMotor.getSensorCollection().isFwdLimitSwitchClosed()) m_armNeedsReset = true;
  }

  @Override
  public void close() {
    m_intakeMotor = null;
    m_armMotor = null;
    m_magazineMotor = null;
  }
}

