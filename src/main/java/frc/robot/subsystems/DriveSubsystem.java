/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  
  //FOR AUTOS ONLY: Sets motors to feed forward or backwards for advanced paths in Auto Mode ONLY
  public boolean ReverseMotors;


  private String SUBSYSTEM_NAME = "Drive Subsystem";

  private DifferentialDrive drivetrain;

  private final WPI_TalonFX LEFT_MASTER_MOTOR = new WPI_TalonFX(Constants.FRONT_LEFT_MOTOR_PORT);
  private final WPI_TalonFX LEFT_REAR_SLAVE = new WPI_TalonFX(Constants.REAR_LEFT_MOTOR_PORT);

  private final WPI_TalonFX RIGHT_MASTER_MOTOR = new WPI_TalonFX(Constants.FRONT_RIGHT_MOTOR_PORT);
  private final WPI_TalonFX RIGHT_REAR_SLAVE = new WPI_TalonFX(Constants.REAR_RIGHT_MOTOR_PORT);

  private final double WHEEL_DIAMETER_METERS = 0.1524;
  private final double TICKS_PER_ROTATION = 2048;
  private final double GEAR_RATIO = 120 / 11;
  private final double TICKS_PER_METER = TICKS_PER_ROTATION * GEAR_RATIO * (1 / Math.PI * WHEEL_DIAMETER_METERS);
  private final double METERS_PER_TICK = 1 / TICKS_PER_METER;

  private final double MIN_TOLERANCE = 1.0;

  private final AHRS NAVX = new AHRS(SPI.Port.kMXP);

  private double speed = 0.0;
  private double turn_scalar = 1.0;
  private double deadband = 0.0; 

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
  public DriveSubsystem(double kP, double kD, double period, double tolerance, double turn_scalar, double deadband) {
      // The PIDController used by the subsystem
      super(new PIDController(kP, 0, kD, period));

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

      // Make mid and rear left motor controllers follow left master
      LEFT_REAR_SLAVE.set(ControlMode.Follower, LEFT_MASTER_MOTOR.getDeviceID());

      // Make mid and rear right motor controllers follow right master
      RIGHT_REAR_SLAVE.set(ControlMode.Follower, RIGHT_MASTER_MOTOR.getDeviceID());

      // Wait for Robot init before finishing DriveSubsystem init
      /* Could do this instead
      try {
        Thread.sleep(7000);
      } catch (Exception e) {
        System.out.println("Exception is: " + e.toString());
        e.printStackTrace();
      } */
      try { Thread.sleep(7000); }
      catch (Exception e) { e.printStackTrace(); }

      // Initialise PID subsystem setpoint and input
      this.resetAngle();
      this.setSetpoint(0);

      // Set drive PID tolerance, minimum is 1 degree
      if (tolerance < this.MIN_TOLERANCE) tolerance = this.MIN_TOLERANCE;
      this.getController().setTolerance(tolerance);

      // Instantiate drive object
      this.drivetrain = new DifferentialDrive(LEFT_MASTER_MOTOR, RIGHT_MASTER_MOTOR);

      // Disable built in deadband application
      this.drivetrain.setDeadband(0);

      this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

      this.turn_scalar = turn_scalar;
      this.deadband = deadband;

      if (Constants.DRIVE_DEBUG) {
        ShuffleboardTab tab = Shuffleboard.getTab(this.SUBSYSTEM_NAME);
        tab.addNumber("Drive Angle", () -> getAngle());
       
      }
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    this.drivetrain.arcadeDrive(this.speed, -output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return this.getAngle();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(Rotation2d.fromDegrees(getHeading()), LEFT_MASTER_MOTOR.getSensorCollection().getIntegratedSensorPosition() * this.METERS_PER_TICK,
                                                          RIGHT_MASTER_MOTOR.getSensorCollection().getIntegratedSensorPosition() * this.METERS_PER_TICK);
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
      this.setSetpoint(this.getController().getSetpoint() + (turn_request * this.turn_scalar));
      this.was_turning = true;
    } else { 
      // When turning is complete, set setpoint to current angle
      if (this.was_turning) {
        this.setSetpoint(this.getAngle());
        this.was_turning = false;
      }
    }

    drivetrain.feed();
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
    
    //These two if statements determine whether the master motors rotate clockwise or counterclockwise based on whether the ReverseMotors boolean is true. This boolean 
    //should be used for reversing the motors IN AUTOS ONLY

    //These reverse standard rotation of the motors, so in theory, they should move in reverse by swapping the negatives and positives of the master motor voltages
    if(ReverseMotors == true){
      LEFT_MASTER_MOTOR.setVoltage(leftVolts);
      RIGHT_MASTER_MOTOR.setVoltage(-rightVolts);
      drivetrain.feed();
    }

    else{
      LEFT_MASTER_MOTOR.setVoltage(-leftVolts);
    RIGHT_MASTER_MOTOR.setVoltage(rightVolts);
    drivetrain.feed();
    }

    
  }
  

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(LEFT_MASTER_MOTOR.getSensorCollection().getIntegratedSensorVelocity(), RIGHT_MASTER_MOTOR.getSensorCollection().getIntegratedSensorVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    LEFT_MASTER_MOTOR.setSelectedSensorPosition(0);
    RIGHT_MASTER_MOTOR.setSelectedSensorPosition(0);
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Returns the currently-estimated pose of the robot
   * @return The pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(NAVX.getAngle(), 360) * -1;
  }
  
  /**
   * Zeros the heading of the robot
   */
  public void zeroHeading() {
    this.resetAngle();
  }

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return NAVX.getRate() * -1;
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
    return ((LEFT_MASTER_MOTOR.getSensorCollection().getIntegratedSensorPosition() * this.METERS_PER_TICK) + 
      (LEFT_MASTER_MOTOR.getSensorCollection().getIntegratedSensorPosition() * this.METERS_PER_TICK) / 2);
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

}
