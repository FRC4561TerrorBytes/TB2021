/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** 
 * Automates the configuration of Talon PID and MotionMagic parameters
 */
public class TalonPIDConfig {
  private static final double MIN_TOLERANCE = 1.0;
  private static final int MIN_MOTION_SMOOTHING = 0;
  private static final int MAX_MOTION_SMOOTHING = 7;
  private static final int PID_SLOT = 0;

  private boolean motionMagic = false;
  private boolean enableSoftLimits = true;

  private boolean sensorPhase = false;
  private boolean invertMotor = false;
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kF = 0.0;
  private double tolerance = 1.0;
  private double lowerLimit = 0.0;
  private double upperLimit = 0.0;

  private double velocityRPM = 1.0;
  private double accelerationRPMPerSec = 1.0;
  private int motionSmoothing = 0;

  /**
   * Create a TalonPIDConfig, without MotionMagic parameters
   * 
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kF feed-forward gain
   * @param tolerance tolerance of PID loop
   * @param lowerLimit lower limit of mechanism
   * @param upperLimit upper limit of mechanism 
   */
  TalonPIDConfig(boolean sensorPhase, boolean invertMotor, 
                  double kP, double kI, double kD, double kF, 
                  double tolerance, double lowerLimit, double upperLimit, boolean enableSoftLimits) {
    this.sensorPhase = sensorPhase;
    this.invertMotor = invertMotor;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.tolerance = tolerance;
    this.lowerLimit = lowerLimit;
    this.upperLimit = upperLimit;
    this.enableSoftLimits = enableSoftLimits;

    if (this.tolerance < MIN_TOLERANCE) this.tolerance = MIN_TOLERANCE;
  }

  /**
   * Create a TalonPIDConfig, without MotionMagic parameters
   * 
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kF feed-forward gain
   * @param tolerance tolerance of PID loop
   */
  TalonPIDConfig(boolean sensorPhase, boolean invertMotor, 
                  double kP, double kI, double kD, double kF, 
                  double tolerance) {
    this.sensorPhase = sensorPhase;
    this.invertMotor = invertMotor;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.tolerance = tolerance;

    this.enableSoftLimits = false;

    if (this.tolerance < MIN_TOLERANCE) this.tolerance = MIN_TOLERANCE;
  }

  /**
   * Create a TalonPIDConfig, with MotionMagic parameters
   * 
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kF feed-forward gain
   * @param tolerance tolerance of PID loop
   * @param velocity MotionMagic cruise velocity in ticks per 100ms
   * @param accelerationRPMPerSec MotionMagic acceleration in ticks per 100ms per sec
   * @param motionSmoothing MotionMagic smoothing factor [0, 7]
   */
  TalonPIDConfig(boolean sensorPhase, boolean invertMotor, 
                  double kP, double kI, double kD, double kF, 
                  double tolerance, double lowerLimit, double upperLimit, boolean enableSoftLimits,
                  double velocityRPM, double accelerationRPMPerSec, int motionSmoothing) {
    this.sensorPhase = sensorPhase;
    this.invertMotor = invertMotor;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.tolerance = tolerance;
    this.lowerLimit = lowerLimit;
    this.upperLimit = upperLimit;
    this.enableSoftLimits = enableSoftLimits;
    
    this.velocityRPM = velocityRPM;
    this.accelerationRPMPerSec = accelerationRPMPerSec;
    this.motionSmoothing = motionSmoothing;

    if (this.tolerance < MIN_TOLERANCE) this.tolerance = MIN_TOLERANCE;

    if (this.motionSmoothing < MIN_MOTION_SMOOTHING) this.motionSmoothing = MIN_MOTION_SMOOTHING;
    if (this.motionSmoothing > MAX_MOTION_SMOOTHING) this.motionSmoothing = MAX_MOTION_SMOOTHING;

    this.motionMagic = true;
  }

  /**
   * Initializes Talon PID and/or MotionMagic parameters
   * @param feedbackDevice Feedback device to use for Talon PID
   */
  public void initializeTalonPID(BaseTalon talon, FeedbackDevice feedbackDevice, boolean forwardLimitSwitch, boolean reverseLimitSwitch) {
    // Reset Talon to default
    talon.configFactoryDefault();

    // Configure feedback sensor
    talon.configSelectedFeedbackSensor(feedbackDevice);
    
    // Configure forward and reverse soft limits
    if (this.enableSoftLimits) {
      talon.configForwardSoftLimitThreshold((int)this.upperLimit);
      talon.configForwardSoftLimitEnable(true);
      talon.configReverseSoftLimitThreshold((int)this.lowerLimit);
      talon.configReverseSoftLimitEnable(true);
    }

    // Configure forward and reverse limit switches if required
    if (forwardLimitSwitch) 
      talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    if (reverseLimitSwitch)
      talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Set sensor phase and invert motor if required
    talon.setSensorPhase(this.sensorPhase);
    talon.setInverted(this.invertMotor);

    // Configure PID values
    talon.config_kP(PID_SLOT, this.kP);
    talon.config_kI(PID_SLOT, this.kI);
    talon.config_kD(PID_SLOT, this.kD);
    talon.config_kF(PID_SLOT, this.kF);
    talon.configAllowableClosedloopError(PID_SLOT, (int)this.tolerance);
    talon.configClosedLoopPeakOutput(PID_SLOT, 1);

    // Configure MotionMagic values
    if (this.motionMagic) {
      talon.configMotionCruiseVelocity(rpmToTicksPer100ms(this.velocityRPM));
      talon.configMotionAcceleration(rpmToTicksPer100ms(this.accelerationRPMPerSec));
      talon.configMotionSCurveStrength(this.motionSmoothing);
    }
  }

   /**
   * Initializes Talon PID and/or MotionMagic parameters
   * @param feedbackDevice Feedback device to use for Talon PID
   */
  public void initializeTalonPID(WPI_TalonFX talon, TalonFXFeedbackDevice feedbackDevice, boolean forwardLimitSwitch, boolean reverseLimitSwitch) {
    // Reset Talon to default
    talon.configFactoryDefault();

    // Configure feedback sensor
    talon.configSelectedFeedbackSensor(feedbackDevice, 0, 0);
    
    // Configure forward and reverse soft limits
    if (this.enableSoftLimits) {
      talon.configForwardSoftLimitThreshold((int)this.upperLimit);
      talon.configForwardSoftLimitEnable(true);
      talon.configReverseSoftLimitThreshold((int)this.lowerLimit);
      talon.configReverseSoftLimitEnable(true);
    }

    // Configure forward and reverse limit switches if required
    if (forwardLimitSwitch) 
      talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    if (reverseLimitSwitch)
      talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Set sensor phase and invert motor if required
    talon.setSensorPhase(this.sensorPhase);
    talon.setInverted(this.invertMotor);

    // Configure PID values
    talon.config_kP(PID_SLOT, this.kP);
    talon.config_kI(PID_SLOT, this.kI);
    talon.config_kD(PID_SLOT, this.kD);
    talon.config_kF(PID_SLOT, this.kF);
    talon.configAllowableClosedloopError(PID_SLOT, (int)this.tolerance);
    talon.configClosedLoopPeakOutput(PID_SLOT, 1);

    // Configure MotionMagic values
    if (this.motionMagic) {
      talon.configMotionCruiseVelocity(rpmToTicksPer100ms(this.velocityRPM));
      talon.configMotionAcceleration(rpmToTicksPer100ms(this.accelerationRPMPerSec));
      talon.configMotionSCurveStrength(this.motionSmoothing);
    }
  }

  private int rpmToTicksPer100ms(double rpm) {
    return (int)((rpm * 4096) / 10);
  }

  /**
   * @return sensor phase
   */
  public boolean getSensorPhase() {
    return sensorPhase;
  }

  /**
   * @return whether motor should be inverted or not
   */
  public boolean getInvertMotor() {
    return invertMotor;
  }

  /**
   * @return proportional gain
   */
  public double getkP() {
    return kP;
  }

  /**
   * @return integral gain
   */
  public double getkI() {
    return kI;
  }

  /**
   * @return derivative gain
   */
  public double getkD() {
    return kD;
  }

  /**
   * @return feed-forward gain
   */
  public double getkF() {
    return kF;
  }

  /**
   * @return PID loop tolerance
   */
  public double getTolerance() {
    return tolerance;
  }

  /**
   * @return lower limit of mechanism
   */
  public double getLowerLimit() {
    return lowerLimit;
  }

  /**
   * @return upper limit of mechanism
   */
  public double getUpperLimit() {
    return upperLimit;
  }

  /**
   * @return MotionMagic cruise velocity in RPM
   */
  public double getVelocityRPM() {
    return velocityRPM;
  }

  /**
   * @return MotionMagic acceleration in RPM per sec
   */
  public double getAccelerationRPMPerSec() {
    return accelerationRPMPerSec;
  }

  /**
   * @return MotionMagic smoothing factor
   */
  public int getMotionSmoothing() {
    return motionSmoothing;
  }
}
