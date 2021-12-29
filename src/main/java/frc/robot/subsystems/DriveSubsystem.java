/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.TractionControlController;


public class DriveSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private WPI_TalonFX lMasterMotor, rMasterMotor;
    private WPI_TalonFX lSlaveMotor, rSlaveMotor;

    private Counter lidar;
    private AHRS navx;

    public Hardware(WPI_TalonFX lMasterMotor, 
                    WPI_TalonFX rMasterMotor, 
                    WPI_TalonFX lSlaveMotor,
                    WPI_TalonFX rSlaveMotor, 
                    Counter lidar,
                    AHRS navx) {
      this.lMasterMotor = lMasterMotor;
      this.rMasterMotor = rMasterMotor;
      this.lSlaveMotor = lSlaveMotor;
      this.rSlaveMotor = rSlaveMotor;

      this.lidar = lidar;
      this.navx = navx;
    }
  }

  private String SUBSYSTEM_NAME = "Drive Subsystem";

  private PIDController m_drivePIDController;
  private TractionControlController m_tractionControlController;

  private WPI_TalonFX m_lMasterMotor;
  private WPI_TalonFX m_lSlaveMotor;

  private WPI_TalonFX m_rMasterMotor;
  private WPI_TalonFX m_rSlaveMotor;

  private AHRS m_navx;

  private Counter m_lidar;
  private final double LIDAR_OFFSET = 10.0;

  private final double TURN_DEADBAND = 0.005;
  private final double WHEEL_DIAMETER_METERS = 0.1524;
  private final double MOTOR_MAX_RPM = 6380;
  private final double TICKS_PER_ROTATION = 2048;
  private final double GEAR_RATIO = 10.90909090; // 120 / 11
  private final double TICKS_PER_METER = (double)(TICKS_PER_ROTATION * GEAR_RATIO) * (double)(1 / (WHEEL_DIAMETER_METERS * Math.PI)); //46644.183
  private final double METERS_PER_TICK = 1 / TICKS_PER_METER; //2.149e-5
  private final double METERS_PER_ROTATION = METERS_PER_TICK * TICKS_PER_ROTATION; //0.04388
  private final double DRIVETRAIN_EFFICIENCY = 0.88;
  private final double MAX_LINEAR_SPEED = Math.floor(((MOTOR_MAX_RPM / 60) * METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY) * 1000) / 1000; //4.106 m/s
  private final double INERTIAL_VELOCITY_THRESHOLD = 0.005;
  private final int INERTIAL_VELOCITY_WINDOW_SIZE = 1;
  private final double[] INERTIAL_VELOCITY_READINGS = new double[INERTIAL_VELOCITY_WINDOW_SIZE];

  private final double TOLERANCE = 0.125;

  private double m_turnScalar = 1.0; 
  private double m_inertialVelocity = 0.0;
  private double m_inertialVelocitySum = 0.0;
  private int m_inertialVelocityIndex = 0;

  private boolean m_wasTurning = false;

  //Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param kP Proportional gain
   * @param kD Derivative gain
   * @param period Time (in seconds) between PID calculations
   * @param tolerance Allowed closed loop error (degrees)
   * @param turn_scalar Scalar for turn input (degrees)
   * @param accelerationLimit Maximum allowed acceleration (m/s^2)
   * @param tractionControlCurve Expression characterising traction of the robot with "X" as the variable
   * @param throttleInputCurve Expression characterising throttle input with "X" as the variable
   */
  public DriveSubsystem(Hardware drivetrainHardware, double kP, double kD, double turn_scalar, double accelerationLimit, String tractionControlCurve, String throttleInputCurve) {
      m_drivePIDController = new PIDController(kP, 0, kD, Constants.ROBOT_LOOP_PERIOD);
      m_tractionControlController = new TractionControlController(MAX_LINEAR_SPEED, accelerationLimit, tractionControlCurve, throttleInputCurve);

      this.m_lMasterMotor = drivetrainHardware.lMasterMotor;
      this.m_rMasterMotor = drivetrainHardware.rMasterMotor;
      this.m_lSlaveMotor = drivetrainHardware.lSlaveMotor;
      this.m_rSlaveMotor = drivetrainHardware.rSlaveMotor;

      this.m_lidar = drivetrainHardware.lidar;
      this.m_navx = drivetrainHardware.navx;

      this.m_turnScalar = turn_scalar;

      // Reset TalonFX settings
      m_lMasterMotor.configFactoryDefault();
      m_lSlaveMotor.configFactoryDefault();
      m_rMasterMotor.configFactoryDefault();
      m_rSlaveMotor.configFactoryDefault();

      // Set all drive motors to brake
      m_lMasterMotor.setNeutralMode(NeutralMode.Brake);
      m_lSlaveMotor.setNeutralMode(NeutralMode.Brake);
      m_rMasterMotor.setNeutralMode(NeutralMode.Brake);
      m_rSlaveMotor.setNeutralMode(NeutralMode.Brake);

      // Invert only right side
      m_lMasterMotor.setInverted(false);
      m_lSlaveMotor.setInverted(false);
      m_rMasterMotor.setInverted(true);
      m_rSlaveMotor.setInverted(true);

      // Make rear left motor controllers follow left master
      m_lSlaveMotor.set(ControlMode.Follower, m_lMasterMotor.getDeviceID());

      // Make rear right motor controllers follow right master
      m_rSlaveMotor.set(ControlMode.Follower, m_rMasterMotor.getDeviceID());

      // Make motors use integrated encoder
      m_lMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      m_rMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

      // Initialise PID subsystem setpoint and input
      resetAngle();
      m_drivePIDController.setSetpoint(0);

      // Set drive PID tolerance
      m_drivePIDController.setTolerance(TOLERANCE);

      m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
      resetOdometry();

      // Configure LIDAR settings
      m_lidar.setMaxPeriod(1.00); //set the max period that can be measured
      m_lidar.setSemiPeriodMode(true); //Set the counter to period measurement
      m_lidar.reset();
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware drivetrainHardware = new Hardware(new WPI_TalonFX(Constants.FRONT_LEFT_MOTOR_PORT),
                                                new WPI_TalonFX(Constants.FRONT_RIGHT_MOTOR_PORT),
                                                new WPI_TalonFX(Constants.REAR_LEFT_MOTOR_PORT),
                                                new WPI_TalonFX(Constants.REAR_RIGHT_MOTOR_PORT),
                                                new Counter(Constants.LIDAR_PORT),
                                                new AHRS(SPI.Port.kMXP));

    return drivetrainHardware;
  }

  /**
   * Create Shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addNumber("Drive Angle", () -> getHeading());
    tab.addNumber("Distance", () -> getLIDAR());
  }

  @Override
  public void periodic() {
    // Display traction control indicator on SmartDashboard
    SmartDashboard.putBoolean("TC", m_tractionControlController.isEnabled());

    updateInertialVelocity();
    
    // Update the odometry in the periodic block
    // Negate gyro angle because gyro is positive going clockwise which doesn't match WPILib convention
    m_odometry.update(Rotation2d.fromDegrees(-getAngle()), 
                      -m_lMasterMotor.getSelectedSensorPosition() * METERS_PER_TICK,
                      m_rMasterMotor.getSelectedSensorPosition() * METERS_PER_TICK);
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

    m_lMasterMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, -turn_request);
    m_rMasterMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, +turn_request);
	}

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param speedRequest Desired speed from -1.0 to 1.0
   * @param turnRequest Turn input from -1.0 to 1.0
   */
  public void teleopPID(double speedRequest, double turnRequest) {
    double currentAngle = getAngle();

    // Start turning if input is greater than deadband
    if (Math.abs(turnRequest) >= TURN_DEADBAND) {
      // Add delta to setpoint scaled by factor
      m_drivePIDController.setSetpoint(currentAngle + (turnRequest * m_turnScalar));
      m_wasTurning = true;
    } else { 
      // When turning is complete, set setpoint to current angle
      if (m_wasTurning) {
        m_drivePIDController.setSetpoint(currentAngle);
        m_wasTurning = false;
      }
    }

    // Calculate next PID turn output
    double turnOutput = m_drivePIDController.calculate(currentAngle);

    // Calculate next motor speed output
    double optimalSpeedOutput = m_tractionControlController.calculate(getInertialVelocity(), speedRequest);

    // Run motors with appropriate values
    m_lMasterMotor.set(ControlMode.PercentOutput, optimalSpeedOutput, DemandType.ArbitraryFeedForward, -turnOutput);
    m_rMasterMotor.set(ControlMode.PercentOutput, optimalSpeedOutput, DemandType.ArbitraryFeedForward, +turnOutput);
  }

  /**
   * Turn robot by angleDelta
   * @param angleDelta degrees to turn robot by
   */
  public void aimToAngle(double angleDelta) {
    double currentAngle = getAngle();

    m_drivePIDController.setSetpoint(currentAngle + angleDelta);

    double turnOutput = m_drivePIDController.calculate(currentAngle);

    m_lMasterMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, -turnOutput);
    m_rMasterMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, +turnOutput);
  }

  /**
   * Turn robot to set angle
   * @param angleSetpoint In degrees [-180, 180]
   * @return True when complete
   */
  public boolean turnToAngle(double angleSetpoint) {
    MathUtil.clamp(angleSetpoint, -180.0, 180.0);
    resetAngle();
    m_drivePIDController.setSetpoint(angleSetpoint);

    double currentAngle = getAngle();
    while (Math.abs(currentAngle) <= Math.abs(angleSetpoint)) {
      double output = m_drivePIDController.calculate(currentAngle, m_drivePIDController.getSetpoint());
      m_lMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, -output);
      m_rMasterMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, output);
      currentAngle = getAngle();
    }

    return true;
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * @param leftVolts  the commanded left output [-12, 12]
   * @param rightVolts  the commanded right output [-12, 12]
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_lMasterMotor.setVoltage(-leftVolts);
    m_rMasterMotor.setVoltage(rightVolts);
  }

  /**
   * Set speed of left and right drive seperately
   * @param leftSpeed speed [-1, 1]
   * @param rightSpeed speed [-1, 1]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_lMasterMotor.set(ControlMode.PercentOutput, leftSpeed, DemandType.ArbitraryFeedForward, 0.0);
    m_rMasterMotor.set(ControlMode.PercentOutput, rightSpeed, DemandType.ArbitraryFeedForward, 0.0);
  }
  
  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_lMasterMotor.getSelectedSensorVelocity() * 10 * METERS_PER_TICK, 
                                            m_rMasterMotor.getSelectedSensorVelocity() * 10 * METERS_PER_TICK);
  }

  /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry() {
    resetAngle();
    m_lMasterMotor.setSelectedSensorPosition(0);
    m_rMasterMotor.setSelectedSensorPosition(0);
    m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
  }

  /**
   * Returns the currently-estimated pose of the robot
   * @return The pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return m_navx.getYaw();
  }
  
  /**
   * Zeros the heading of the robot
   */
  public void zeroHeading() {
    resetAngle();
  }

  /**
   * Reset left and right drive
   */
  public void resetEncoders() {
    m_lMasterMotor.setSelectedSensorPosition(0);
    m_rMasterMotor.setSelectedSensorPosition(0);
  }

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_navx.getRate();
  }

  /**
   * Update Robot's current inertial velocity (m/s) using a moving average
   */
  public void updateInertialVelocity() {
    // Remove oldest reading from running sum
    m_inertialVelocitySum = m_inertialVelocitySum - INERTIAL_VELOCITY_READINGS[m_inertialVelocityIndex];

    // Get latest reading from NAVX
    double latestReading = MathUtil.clamp(m_navx.getVelocityY(), -MAX_LINEAR_SPEED, +MAX_LINEAR_SPEED);

    // Add latest reading to array
    INERTIAL_VELOCITY_READINGS[m_inertialVelocityIndex] = latestReading;

    // Add latest reading to running sum of readings
    m_inertialVelocitySum += latestReading;

    // Increment index, wrapping around to zero if it surpasses window size
    m_inertialVelocityIndex = (m_inertialVelocityIndex + 1) % INERTIAL_VELOCITY_WINDOW_SIZE;
    
    // Update inertial velocity variable with latest average
    m_inertialVelocity = m_inertialVelocitySum / INERTIAL_VELOCITY_WINDOW_SIZE;
  }

  /**
   * Returns inertial velocity of the robot.
   * @return Velocity of the robot as measured by the NAVX
   */
  public double getInertialVelocity() {
    // Return the latest moviing average, ignoring really small values
    return (Math.abs(m_inertialVelocity) >= INERTIAL_VELOCITY_THRESHOLD) ? 
            m_inertialVelocity : 
            0;
  }

  /**
   * Converts encoder position to meters
   * @param ticks Encoder position
   * @return Return distance in meters
   */
  public double getDistance(double ticks) {
    return ticks * METERS_PER_TICK;
  }

  /**
   * Gets the average distance of the two encoders.
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (((m_lMasterMotor.getSensorCollection().getIntegratedSensorPosition() * METERS_PER_TICK) + 
              (m_lMasterMotor.getSensorCollection().getIntegratedSensorPosition() * METERS_PER_TICK)) / 2);
  }

  /**
   * Stop drivetrain
   */
  public void stop() {
    m_lMasterMotor.set(ControlMode.PercentOutput, 0.0);
    m_rMasterMotor.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_tractionControlController.toggle();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_tractionControlController.disable();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_tractionControlController.enable();
  }

  /**
   * Get DriveSubsystem angle as detected by the navX MXP
   * @return Total accumulated yaw angle
   */
  public double getAngle() {
    return m_navx.getAngle();
  }

  /**
   * Reset DriveSubsystem navX MXP yaw angle
   */
  public void resetAngle() {
    m_navx.reset();
  }

  /**
   * Get Distance from LIDAR sensor
   * @return distance in Meters
   */
  public double getLIDAR() {
    if(m_lidar.get() < 1)
      return 0;
    else
      return ((m_lidar.getPeriod()*1000000.0/10.0) - LIDAR_OFFSET) / 100.0; //convert to distance. sensor is high 10 us for every centimeter. 
  }

  /**
   * Set setpoint for drive PID
   * @param setpoint angle in degrees
   */
  public void setDrivePIDSetpoint(double setpoint) {
    m_drivePIDController.setSetpoint(setpoint);
  }

  /**
   * Get setpoint for drive PID
   * @return current setpoint in degrees
   */
  public double getDrivePIDSetpoint() {
    return m_drivePIDController.getSetpoint();
  }

  @Override
  public void close() {
    m_lMasterMotor = null;
    m_rMasterMotor = null;
    m_lSlaveMotor = null;
    m_rSlaveMotor = null;
    m_lidar = null;
    m_navx = null;
  }
}
