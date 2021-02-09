/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;


public class DriveSubsystem extends PIDSubsystem {

  private String SUBSYSTEM_NAME = "Drive Subsystem";

  private DifferentialDrive drivetrain;

  private final WPI_TalonFX LEFT_MASTER_MOTOR = new WPI_TalonFX(Constants.FRONT_LEFT_MOTOR_PORT);
  private final WPI_TalonFX LEFT_REAR_SLAVE = new WPI_TalonFX(Constants.REAR_LEFT_MOTOR_PORT);

  private final WPI_TalonFX RIGHT_MASTER_MOTOR = new WPI_TalonFX(Constants.FRONT_RIGHT_MOTOR_PORT);
  private final WPI_TalonFX RIGHT_REAR_SLAVE = new WPI_TalonFX(Constants.REAR_RIGHT_MOTOR_PORT);

  private final double WHEEL_DIAMETER_METERS = 0.1524;
  private final double MOTOR_MAX_RPM = 6380;
  private final double TICKS_PER_ROTATION = 2048;
  private final double GEAR_RATIO = 10.90909090; // 120 / 11
  private final double TICKS_PER_METER = (double)(TICKS_PER_ROTATION * GEAR_RATIO) * (double)(WHEEL_DIAMETER_METERS * Math.PI);
  private final double METERS_PER_TICK = 1 / TICKS_PER_METER;
  private final double METERS_PER_ROTATION = METERS_PER_TICK * TICKS_PER_ROTATION;
  private final double DRIVETRAIN_EFFICIENCY = 0.85;
  private final double MAX_LINEAR_SPEED = (MOTOR_MAX_RPM / 60) * METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY;
  private final double OPTIMAL_SLIP_RATIO = 0.05;

  private final double MIN_TOLERANCE = 0.125;

  private final AHRS NAVX = new AHRS(SPI.Port.kMXP);

  private double speed = 0.0;
  private double turn_scalar = 1.0;
  private double deadband = 0.0; 
  private double output = 0.0;

  private boolean was_turning = false;

  //Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param kP Proportional gain
   * @param kD Derivative gain
   * @param period Time (in seconds) between PID calculations
   * @param tolerance Allowed closed loop error (degrees)
   * @param turn_scalar Turn sensitivity
   * @param deadband Deadzone for joystick
   */
  public DriveSubsystem(double kP, double kD, double tolerance, double turn_scalar, double deadband) {
      // The PIDController used by the subsystem
      super(new PIDController(kP, 0, kD));

      // Reset master TalonFX settings
      LEFT_MASTER_MOTOR.configFactoryDefault();
      RIGHT_MASTER_MOTOR.configFactoryDefault();

      // Set all drive motors to brake
      LEFT_MASTER_MOTOR.setNeutralMode(NeutralMode.Brake);
      LEFT_REAR_SLAVE.setNeutralMode(NeutralMode.Brake);
      RIGHT_MASTER_MOTOR.setNeutralMode(NeutralMode.Brake);
      RIGHT_REAR_SLAVE.setNeutralMode(NeutralMode.Brake);

      // Do NOT invert motors
      LEFT_MASTER_MOTOR.setInverted(false);
      LEFT_REAR_SLAVE.setInverted(false);
      RIGHT_MASTER_MOTOR.setInverted(false);
      RIGHT_REAR_SLAVE.setInverted(false);

      // Make rear left motor controllers follow left master
      LEFT_REAR_SLAVE.set(ControlMode.Follower, LEFT_MASTER_MOTOR.getDeviceID());

      // Make rear right motor controllers follow right master
      RIGHT_REAR_SLAVE.set(ControlMode.Follower, RIGHT_MASTER_MOTOR.getDeviceID());

      // Make motors use integrated encoder
      LEFT_MASTER_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      RIGHT_MASTER_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

      // Wait for NAVX init before finishing DriveSubsystem init
      try { Thread.sleep(3000); }
      catch (Exception e) { e.printStackTrace(); }

      // Initialise PID subsystem setpoint and input
      this.resetAngle();
      this.setSetpoint(0);

      // Set drive PID tolerance, minimum is 0.125 degree
      if (tolerance < this.MIN_TOLERANCE) tolerance = this.MIN_TOLERANCE;
      this.getController().setTolerance(tolerance);

      // Instantiate drive object
      this.drivetrain = new DifferentialDrive(LEFT_MASTER_MOTOR, RIGHT_MASTER_MOTOR);

      // Disable built in deadband application
      this.drivetrain.setDeadband(0);

      this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
      this.resetOdometry();

      this.turn_scalar = turn_scalar;
      this.deadband = deadband;

      if (Constants.DRIVE_DEBUG) {
        ShuffleboardTab tab = Shuffleboard.getTab(this.SUBSYSTEM_NAME);
        tab.addNumber("Drive Angle", () -> getHeading());
        tab.addNumber("Drive PID Output", () -> this.output);
      }
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here

    // Apply basic traction control when going straight
    if (!this.was_turning) {
      // Get average linear wheel speeds
      DifferentialDriveWheelSpeeds wheelSpeeds = this.getWheelSpeeds();
      double averageWheelSpeed = Math.abs((wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond) / 2);
      double inertialVelocity = this.getInertialVelocity();
      double currentSlipRatio = (averageWheelSpeed > inertialVelocity) ?
																(averageWheelSpeed - inertialVelocity) / averageWheelSpeed :
																0;

      // If current slip ratio is greater than optimal then wheel is slipping excessively
      if (currentSlipRatio >= this.OPTIMAL_SLIP_RATIO) {
				// Calculate optimal speed based on optimal slip ratio
				double optimalSpeed = (inertialVelocity != 0) ? 
															(-inertialVelocity / (this.OPTIMAL_SLIP_RATIO - 1)) / this.MAX_LINEAR_SPEED : 
															(this.OPTIMAL_SLIP_RATIO * this.speed);
        this.setSpeed(Math.copySign(optimalSpeed, this.speed));
      }
    }

    this.drivetrain.arcadeDrive(this.speed, -output, false);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return this.getAngle();
  }

  @Override
  public void periodic() {
    if (this.isEnabled()) {
      this.useOutput(this.getController().calculate(this.getMeasurement(), this.getSetpoint()), this.getSetpoint());
    }
    // Update the odometry in the periodic block
    // Negate gyro angle because gyro is positive going clockwise which doesn't match WPILib convention
    this.odometry.update(Rotation2d.fromDegrees(-this.getAngle()), LEFT_MASTER_MOTOR.getSelectedSensorPosition() * this.METERS_PER_TICK,
                                                          RIGHT_MASTER_MOTOR.getSelectedSensorPosition() * this.METERS_PER_TICK);
  }

  /**
   * Call this repeatedly to drive without PID during teleoperation
   * @param speed Desired speed from -1.0 to 1.0
   * @param turn_request Turn input from -1.0 to 1.0
   * @param power exponent for drive response curve. 1 is linear response
   */
	public void teleop(double speed, double turn_request, int power) {
    speed = Math.copySign(Math.pow(speed, power), speed);
    turn_request = Math.copySign(Math.pow(turn_request, power), turn_request);

    this.drivetrain.curvatureDrive(speed, -turn_request, true);
	}

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param speed Desired speed from -1.0 to 1.0
   * @param turn_request Turn input from -1.0 to 1.0
   * @param power exponent for drive response curve. 1 is linear response
   */
  public void teleopPID(double speed, double turn_request, int power) {
    speed = Math.copySign(Math.pow(speed, power), speed);
    turn_request = Math.copySign(Math.pow(turn_request, power), turn_request);

    // Set drive speed if it is more than the deadband
    if (Math.abs(speed) >= this.deadband) this.setSpeed(speed);
    else this.stop();

    // Start turning if input is greater than deadband
    if (Math.abs(turn_request) >= this.deadband) {
      // Add delta to setpoint scaled by factor
      this.setSetpoint(this.getMeasurement() + (turn_request * this.turn_scalar));
      this.was_turning = true;
    } else { 
      // When turning is complete, set setpoint to current angle
      if (this.was_turning) {
        this.setSetpoint(this.getMeasurement());
        this.was_turning = false;
      }
    }

    this.drivetrain.feed();
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * @param leftVolts  the commanded left output
   * @param rightVolts  the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    LEFT_MASTER_MOTOR.setVoltage(-leftVolts);
    RIGHT_MASTER_MOTOR.setVoltage(rightVolts);
    drivetrain.feed();
  }

  /**
   * Sets maximum output of drivetrain
   * @param maxOutput
   */
  public void setMaxOutput(double maxOutput) {
    this.drivetrain.setMaxOutput(maxOutput);
  }
  
  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(LEFT_MASTER_MOTOR.getSelectedSensorVelocity() * 10 * METERS_PER_TICK, 
      RIGHT_MASTER_MOTOR.getSelectedSensorVelocity() * 10 * METERS_PER_TICK);
  }

  /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry() {
    this.resetAngle();
    LEFT_MASTER_MOTOR.setSelectedSensorPosition(0);
    RIGHT_MASTER_MOTOR.setSelectedSensorPosition(0);
    this.odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
  }

  /**
   * Returns the currently-estimated pose of the robot
   * @return The pose
   */
  public Pose2d getPose() {
    return this.odometry.getPoseMeters();
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return this.NAVX.getYaw();
  }
  
  /**
   * Zeros the heading of the robot
   */
  public void zeroHeading() {
    this.resetAngle();
  }

  /**
   * Reset left and right drive
   */
  public void resetEncoders() {
    LEFT_MASTER_MOTOR.setSelectedSensorPosition(0);
    RIGHT_MASTER_MOTOR.setSelectedSensorPosition(0);
  }

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -NAVX.getRate();
  }

  /**
   * Returns inertial velocity of the robot.
   * @return Velocity of the robot as measured by the NAVX
   */
  public double getInertialVelocity() {
    return (NAVX.isMoving()) ? Math.sqrt(Math.pow(NAVX.getVelocityX(), 2) + Math.pow(NAVX.getVelocityY(), 2)) : 0;
  }

  /**
   * Converts encoder position to meters
   * @param ticks Encoder position
   * @return Return distance in meters
   */
  public double getDistance(double ticks) {
    return ticks * this.METERS_PER_TICK;
  }

  /**
   * Gets the average distance of the two encoders.
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (((LEFT_MASTER_MOTOR.getSensorCollection().getIntegratedSensorPosition() * this.METERS_PER_TICK) + 
      (LEFT_MASTER_MOTOR.getSensorCollection().getIntegratedSensorPosition() * this.METERS_PER_TICK)) / 2);
  }

  /**
   * Stop drivetrain
   */
  public void stop() {
    this.setSpeed(0);
  }

  /**
   * Set DriveSubsystem speed
   * @param speed Desired speed from -1.0 to 1.0
   */
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  /**
   * Get DriveSubsystem speed
   * @return Speed
   */
  public double getSpeed() {
    return this.speed;
  }

  /**
   * Get DriveSubsystem angle as detected by the navX MXP
   * @return Total accumulated yaw angle
   */
  public double getAngle() {
    return NAVX.getAngle();
  }

  /**
   * Reset DriveSubsystem navX MXP yaw angle
   */
  public void resetAngle() {
    NAVX.reset();
  }

  /**
   * Invert Drive Motors
   * @param invert set false for Teleop
   */
  public void invertMotors(boolean invert) {
    LEFT_MASTER_MOTOR.setInverted(invert);
    LEFT_REAR_SLAVE.setInverted(invert);
    RIGHT_MASTER_MOTOR.setInverted(invert);
    RIGHT_REAR_SLAVE.setInverted(invert);
  }

}
