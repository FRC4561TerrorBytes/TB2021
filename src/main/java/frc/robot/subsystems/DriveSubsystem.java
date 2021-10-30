/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    public DifferentialDrive drivetrain;
    public WPI_TalonFX leftMasterMotor, rightMasterMotor;
    public WPI_TalonFX leftSlaveMotor, rightSlaveMotor;

    public Counter lidar;
    public AHRS navx;

    public Hardware(WPI_TalonFX leftMasterMotor, 
                    WPI_TalonFX rightMasterMotor, 
                    WPI_TalonFX leftSlaveMotor,
                    WPI_TalonFX rightSlaveMotor, 
                    Counter lidar,
                    AHRS navx) {
      this.leftMasterMotor = leftMasterMotor;
      this.rightMasterMotor = rightMasterMotor;
      this.leftSlaveMotor = leftSlaveMotor;
      this.rightSlaveMotor = rightSlaveMotor;

      this.lidar = lidar;
      this.navx = navx;

      this.drivetrain = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
    }

    public Hardware(DifferentialDrive drivetrain,
                    WPI_TalonFX leftMasterMotor, 
                    WPI_TalonFX rightMasterMotor, 
                    WPI_TalonFX leftSlaveMotor,
                    WPI_TalonFX rightSlaveMotor, 
                    Counter lidar,
                    AHRS navx) {
      this.drivetrain = drivetrain;
      this.leftMasterMotor = leftMasterMotor;
      this.rightMasterMotor = rightMasterMotor;
      this.leftSlaveMotor = leftSlaveMotor;
      this.rightSlaveMotor = rightSlaveMotor;

      this.lidar = lidar;
      this.navx = navx;
    }
  }

  private String SUBSYSTEM_NAME = "Drive Subsystem";

  private ShuffleboardTab m_shuffleboardTab;

  private DifferentialDrive m_drivetrain;
  private PIDController m_drivePIDController;

  private WPI_TalonFX m_leftMasterMotor;
  private WPI_TalonFX m_leftSlaveMotor;

  private WPI_TalonFX m_rightMasterMotor;
  private WPI_TalonFX m_rightSlaveMotor;

  private AHRS m_navx;

  private Counter m_lidar;
  private final double LIDAR_OFFSET = 10.0;

  private final double WHEEL_DIAMETER_METERS = 0.1524;
  private final double MOTOR_MAX_RPM = 6380;
  private final double TICKS_PER_ROTATION = 2048;
  private final double GEAR_RATIO = 10.90909090; // 120 / 11
  private final double TICKS_PER_METER = (double)(TICKS_PER_ROTATION * GEAR_RATIO) * (double)(1 / (WHEEL_DIAMETER_METERS * Math.PI)); //46644.183
  private final double METERS_PER_TICK = 1 / TICKS_PER_METER; //2.149e-5
  private final double METERS_PER_ROTATION = METERS_PER_TICK * TICKS_PER_ROTATION; //0.04388
  private final double DRIVETRAIN_EFFICIENCY = 0.88;
  private final double MAX_LINEAR_SPEED = Math.floor(((MOTOR_MAX_RPM / 60) * METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY) * 1000) / 1000; //4.106 m/s
  private final double OPTIMAL_SLIP_RATIO = 0.03;
  private final double INERTAL_VELOCITY_THRESHOLD = 0.005;
  private final int INERTIAL_VELOCITY_WINDOW_SIZE = 50;
  private final double[] INERTIAL_VELOCITY_READINGS = new double[INERTIAL_VELOCITY_WINDOW_SIZE];

  private final double MIN_TOLERANCE = 0.125;

  private double m_speed = 0.0;
  private double m_turnScalar = 1.0;
  private double m_deadband = 0.0; 
  private double m_output = 0.0;
  private double m_inertialVelocity = 0.0;
  private double m_inertialVelocitySum = 0.0;
  private int m_inertialVelocityIndex = 0;
  private double m_tolerance = 0;

  private boolean m_wasTurning = false;

  private HashMap<Double, Double> m_tractionControlMap = new HashMap<Double, Double>();

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
   * @param turn_scalar Turn sensitivity
   * @param deadband Deadzone for joystick
   * @param tractionControlCurve Expression characterising traction of the robot with "X" as the variable
   */
  public DriveSubsystem(Hardware drivetrainHardware, double kP, double kD, double tolerance, double turn_scalar, double deadband, String tractionControlCurve) {
      // The PIDController used by the subsystem
      m_drivePIDController = new PIDController(kP, 0, kD);

      this.m_leftMasterMotor = drivetrainHardware.leftMasterMotor;
      this.m_rightMasterMotor = drivetrainHardware.rightMasterMotor;
      this.m_leftSlaveMotor = drivetrainHardware.leftSlaveMotor;
      this.m_rightSlaveMotor = drivetrainHardware.rightSlaveMotor;

      this.m_lidar = drivetrainHardware.lidar;
      this.m_navx = drivetrainHardware.navx;

       
      this.m_drivetrain = drivetrainHardware.drivetrain;

      // Disable built in deadband application
      this.m_drivetrain.setDeadband(0);

      // Reset master TalonFX settings
      m_leftMasterMotor.configFactoryDefault();
      m_rightMasterMotor.configFactoryDefault();

      // Set all drive motors to brake
      m_leftMasterMotor.setNeutralMode(NeutralMode.Brake);
      m_leftSlaveMotor.setNeutralMode(NeutralMode.Brake);
      m_rightMasterMotor.setNeutralMode(NeutralMode.Brake);
      m_rightSlaveMotor.setNeutralMode(NeutralMode.Brake);

      // Do NOT invert motors
      m_leftMasterMotor.setInverted(false);
      m_leftSlaveMotor.setInverted(false);
      m_rightMasterMotor.setInverted(false);
      m_rightSlaveMotor.setInverted(false);

      // Make rear left motor controllers follow left master
      m_leftSlaveMotor.set(ControlMode.Follower, m_leftMasterMotor.getDeviceID());

      // Make rear right motor controllers follow right master
      m_rightSlaveMotor.set(ControlMode.Follower, m_rightMasterMotor.getDeviceID());

      // Make motors use integrated encoder
      m_leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      m_rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

      ScriptEngine jsEngine = new ScriptEngineManager().getEngineByName("JavaScript");
      for (int i = 0; i < MAX_LINEAR_SPEED * 1000; i++) {
        double key = (double)i / 1000;
        try {
          double value = (double)jsEngine.eval(tractionControlCurve.replace("X", String.valueOf(key)));
          value = MathUtil.clamp(value, 0, 1.0);
          m_tractionControlMap.put(key, value);
        } catch (ScriptException e) {
          e.printStackTrace();
        }
      }

      // Initialise PID subsystem setpoint and input
      resetAngle();
      m_drivePIDController.setSetpoint(0);

      // Set drive PID tolerance, minimum is 0.125 degree
      m_tolerance = tolerance;
      if (m_tolerance < MIN_TOLERANCE) m_tolerance = MIN_TOLERANCE;
      m_drivePIDController.setTolerance(m_tolerance);

      m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
      resetOdometry();

      this.m_turnScalar = turn_scalar;
      this.m_deadband = deadband;

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
    tab.addNumber("Drive PID Output", () -> m_output);
    tab.addNumber("Distance", () -> getLIDAR());
  }

  @Override
  public void periodic() {
    this.updateInertialVelocity();
    
    // Update the odometry in the periodic block
    // Negate gyro angle because gyro is positive going clockwise which doesn't match WPILib convention
    m_odometry.update(Rotation2d.fromDegrees(-getAngle()), 
                      -m_leftMasterMotor.getSelectedSensorPosition() * METERS_PER_TICK,
                      m_rightMasterMotor.getSelectedSensorPosition() * METERS_PER_TICK);
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

    this.m_drivetrain.curvatureDrive(speed, -turn_request, true);
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
    if (Math.abs(speed) >= m_deadband) setSpeed(speed);
    else this.stop();

    // Start turning if input is greater than deadband
    if (Math.abs(turn_request) >= m_deadband) {
      // Add delta to setpoint scaled by factor
      m_drivePIDController.setSetpoint(getAngle() + (turn_request * m_turnScalar));
      m_wasTurning = true;
    } else { 
      // When turning is complete, set setpoint to current angle
      if (this.m_wasTurning) {
        m_drivePIDController.setSetpoint(getAngle());
        m_wasTurning = false;
      }
    }

    double output = m_drivePIDController.calculate(getAngle(), m_drivePIDController.getSetpoint());
    // Truncate values to 3 decimal places
    double inertialVelocity = Math.floor(getInertialVelocity() * 1000) / 1000;
    // Calculate optimal speed based on optimal slip ratio
    double optimalMotorOutput = (inertialVelocity != 0) ? 
                          m_tractionControlMap.get(inertialVelocity) : 
                          OPTIMAL_SLIP_RATIO * m_speed;
    optimalMotorOutput = Math.copySign(optimalMotorOutput, speed);

    this.m_drivetrain.arcadeDrive(optimalMotorOutput, -output, false);
  }

  /**
   * Turn robot to set angle
   * @param angle In degrees
   * @return True when complete
   */
  public void turnToAngle(double angle) {
    resetAngle();
    m_drivePIDController.setSetpoint(angle);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * @param leftVolts  the commanded left output [-12, 12]
   * @param rightVolts  the commanded right output [-12, 12]
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMasterMotor.setVoltage(-leftVolts);
    m_rightMasterMotor.setVoltage(rightVolts);
    m_drivetrain.feed();
  }

  /**
   * Set speed of left and right drive seperately
   * @param leftSpeed speed [-1, 1]
   * @param rightSpeed speed [-1, 1]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drivetrain.tankDrive(leftSpeed, rightSpeed, false);
  }

  /**
   * Sets maximum output of drivetrain
   * @param maxOutput
   */
  public void setMaxOutput(double maxOutput) {
    this.m_drivetrain.setMaxOutput(maxOutput);
  }
  
  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftMasterMotor.getSelectedSensorVelocity() * 10 * METERS_PER_TICK, 
      m_rightMasterMotor.getSelectedSensorVelocity() * 10 * METERS_PER_TICK);
  }

  /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry() {
    this.resetAngle();
    m_leftMasterMotor.setSelectedSensorPosition(0);
    m_rightMasterMotor.setSelectedSensorPosition(0);
    this.m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
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
    m_leftMasterMotor.setSelectedSensorPosition(0);
    m_rightMasterMotor.setSelectedSensorPosition(0);
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
    double latestReading = Math.min(Math.sqrt(Math.pow(m_navx.getVelocityX(), 2) + Math.pow(m_navx.getVelocityY(), 2)), MAX_LINEAR_SPEED);

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
    return (m_inertialVelocity >= INERTAL_VELOCITY_THRESHOLD) ? 
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
    return (((m_leftMasterMotor.getSensorCollection().getIntegratedSensorPosition() * METERS_PER_TICK) + 
      (m_leftMasterMotor.getSensorCollection().getIntegratedSensorPosition() * METERS_PER_TICK)) / 2);
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
    this.m_speed = speed;
  }

  /**
   * Get DriveSubsystem speed
   * @return Speed
   */
  public double getSpeed() {
    return this.m_speed;
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
   * Returns tolerance
   * @return tolerance
   */
  public double getTolerance() {
    return m_tolerance;
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
   * Get DriveSubsystem PID controller
   * @return PID Controller
   */
  public PIDController getDrivePIDController() {
    return m_drivePIDController;
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
    m_drivetrain = null;
    m_leftMasterMotor = null;
    m_rightMasterMotor = null;
    m_leftSlaveMotor = null;
    m_rightSlaveMotor = null;
    m_lidar = null;
    m_navx = null;
  }
}
