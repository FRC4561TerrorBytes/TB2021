/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.TalonPIDConfig;
import frc.robot.VisionData;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private WPI_TalonFX flywheelMasterMotor, flywheelSlaveMotor;
    private WPI_TalonSRX hoodMotor, turretMotor;

    public Hardware(WPI_TalonFX flywheelMasterMotor, WPI_TalonFX flywheelSlaveMotor, WPI_TalonSRX hoodMotor, WPI_TalonSRX turretMotor) {
      this.flywheelMasterMotor = flywheelMasterMotor;
      this.flywheelSlaveMotor = flywheelSlaveMotor;
      this.hoodMotor = hoodMotor;
      this.turretMotor = turretMotor;
    }
  }

  private final String SUBSYSTEM_NAME = "Shooter Subsystem";

  // Config values and create motor objects for the flywheel
  private static class Flywheel {
    private static final double MAX_SPEED_RPM = 6380;
    private static final int TICKS_PER_ROTATION = 2048;
    private static WPI_TalonFX masterMotor;
    private static WPI_TalonFX slaveMotor;
    private static TalonPIDConfig masterConfig;
    private static boolean isRunning = false;

    private static double rpmToTicksPer100ms(double speed) {
      return (speed * TICKS_PER_ROTATION) / 600;
    }

    private static double ticksToRPM(double speed) {
      return (speed * 600) / TICKS_PER_ROTATION;
    }
  }

  // Config values and create motor object for the hood
  private static class Hood {
    private static boolean needsReset = true;
    private static int topPosition = Constants.HOOD_TOP_POSITION;
    private static double bottomPosition = Constants.HOOD_BOTTOM_POSITION;
    private static WPI_TalonSRX motor;
    private static TalonPIDConfig config;
  }
// Config values and create motor object for the turret
  private static class Turret {
    private static boolean needsReset = true;
    private static int leftPosition = Constants.TURRET_FRONT_LIMIT_POSITION;
    private static int straightPosition = Constants.TURRET_STRAIGHT_POSITION;
    private static int rightPosition = Constants.TURRET_BACK_LIMIT_POSITION;
    private static final int TICKS_PER_ROTATION = Constants.CTRE_MAG_ENCODER_TICKS_PER_ROTATION;
    private static final double GEAR_RATIO = Constants.TURRET_GEAR_RATIO_AFTER_ENCODER;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_ROTATION * GEAR_RATIO) / 360;
    private static WPI_TalonSRX motor;
    private static TalonPIDConfig config;

    private static double degreesToTicks(double degrees) {
      return (TICKS_PER_DEGREE * degrees);
    }

    private static double ticksToDegrees(double ticks) {
      return (ticks / TICKS_PER_DEGREE);
    }
  }

  /**
  * Create an instance of ShooterSubsystem
  * <p>
  * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
  * <p>
  * NOTE: ASSUMES HOOD STARTS IN LOWEST POSITION!
  * <p>
  * NOTE: ASSUMES TURRET STARTS IN RIGHTMOST POSITION!
  * <p>
   * @param flywheelConfig PID config for Flywheel
   * @param hoodConfig PID config for Hood
   * @param turretConfig PID config for Turret
   */
  public ShooterSubsystem(Hardware shooterHardware, TalonPIDConfig flywheelMasterConfig, TalonPIDConfig hoodConfig, TalonPIDConfig turretConfig) {
    Flywheel.masterMotor = shooterHardware.flywheelMasterMotor;
    Flywheel.slaveMotor = shooterHardware.flywheelSlaveMotor;
    Hood.motor = shooterHardware.hoodMotor;
    Turret.motor = shooterHardware.turretMotor;

    Flywheel.masterConfig = flywheelMasterConfig;
    Hood.config = hoodConfig;
    Turret.config = turretConfig;

    // Initialize config for flywheel PID
    Flywheel.masterConfig.initializeTalonPID(Flywheel.masterMotor, TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(), false, false);
    Flywheel.slaveMotor.set(ControlMode.Follower, Flywheel.masterMotor.getDeviceID());
    Flywheel.slaveMotor.setInverted(true);

    // Initialize config for hood and turret PID
    Hood.config.initializeTalonPID(Hood.motor, FeedbackDevice.CTRE_MagEncoder_Relative, true, false);
    Turret.config.initializeTalonPID(Turret.motor, FeedbackDevice.CTRE_MagEncoder_Relative, true, true);

    // Reset encoders to 0 on initialisation
    resetEncoder(Flywheel.masterMotor);
    resetEncoder(Hood.motor);
    
    // Initialize Turret to 0
    moveTurretPID(Constants.TURRET_FRONT_LIMIT_POSITION);
    
    // Move turret to vision target
    VisionData.setXAngleListener(new TableEntryListener(){
      @Override
      public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
          int flags) {
        turretVisionPID();
      }
    });
  }

  /**
   * Initialize hardware devices for shooter subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(new WPI_TalonFX(Constants.FLYWHEEL_MASTER_MOTOR_PORT),
                                            new WPI_TalonFX(Constants.FLYWHEEL_SLAVE_MOTOR_PORT),
                                            new WPI_TalonSRX(Constants.HOOD_MOTOR_PORT),
                                            new WPI_TalonSRX(Constants.TURRET_MOTOR_PORT));
    return shooterHardware;
  }

  /**
   * Create shuffleboard tab for this subsystem and display values
   */
  public void shuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    Shuffleboard.getTab("SmartDashboard").addBoolean("Flywheel at Speed?", () -> isFlywheelAtSpeed());
    tab.addNumber("Flywheel Motor Output", () -> Flywheel.masterMotor.getMotorOutputPercent());
    tab.addNumber("Flywheel Current", () -> Flywheel.masterMotor.getSupplyCurrent());
    tab.addNumber("Flywheel Motor Velocity", () -> Flywheel.ticksToRPM(Flywheel.masterMotor.getSelectedSensorVelocity()));
    tab.addNumber("Flywheel Motor Setpoint", () -> Flywheel.ticksToRPM(Flywheel.masterMotor.getClosedLoopTarget()));
    tab.addNumber("Flywheel Error", () -> flywheelError());

    tab.addNumber("Hood Motor Output", () -> Hood.motor.getMotorOutputPercent());
    tab.addNumber("Hood Motor Current", () -> Hood.motor.getSupplyCurrent());
    tab.addNumber("Hood Encoder Position", () -> Hood.motor.getSelectedSensorPosition());
    tab.addNumber("Hood Error", () -> Hood.motor.getClosedLoopError());
    
    tab.addNumber("Turret Motor Output", () -> Turret.motor.getMotorOutputPercent());
    tab.addNumber("Turret Motor Current", () -> Turret.motor.getSupplyCurrent());
    tab.addNumber("Turret Encoder Position", () -> Turret.motor.getSelectedSensorPosition());
    tab.addNumber("Turret Encoder Setpoint", () -> Turret.motor.getClosedLoopTarget());
    tab.addNumber("Turret Error", () -> Turret.motor.getClosedLoopError());
  }
  
  /**
   * Reset the Turret and Hood setpoints
   */
  public void reset() {
    Turret.motor.setSelectedSensorPosition(Constants.TURRET_STRAIGHT_POSITION);
    moveHoodPID(Hood.bottomPosition);
  }

  /**
   * Moves hood to position
   * @param setpoint input position to move to (in ticks)
   */
  public void moveHoodPID(double setpoint) {
    // Normalise setpoint
    if (setpoint < Hood.topPosition) setpoint = Hood.topPosition;
    else if (setpoint > Hood.bottomPosition) setpoint = Hood.bottomPosition;

    // Move hood toward setpoint
    Hood.motor.set(ControlMode.MotionMagic, setpoint);
  }

  /**
   * Toggles hood between top and bottom positions
   */
  public void toggleHoodPosition() {
    if (Hood.motor.getClosedLoopTarget() == Constants.HOOD_TOP_POSITION) {
      moveHoodPID(Constants.HOOD_BOTTOM_POSITION);
    } else if (Hood.motor.getClosedLoopTarget() == Constants.HOOD_BOTTOM_POSITION) {
      moveHoodPID(Constants.HOOD_TOP_POSITION);
    }
  }

  /**
   * Move hood at specified speed
   * @param speed speed to move hood at [-1, 1]
   */
  public void hoodManual(double speed) {
    Hood.motor.set(speed);
  }

  /**
   * Moves turret to position
   * @param setpoint inputs position to move to (in ticks)
   */
  public void moveTurretPID(double setpoint) {
    // Normalise setpoint
    if (setpoint < Turret.leftPosition) setpoint = Turret.leftPosition;
    else if (setpoint > Turret.rightPosition) setpoint = Turret.rightPosition;

    // Move turret toward setpoint
    Turret.motor.set(ControlMode.MotionMagic, setpoint);
  }

  /**
   * Automatically aim at vision target
   */
  public void turretVisionPID() {
    if (VisionData.isReady() && VisionData.isDetected()) {
      double ticks = Turret.degreesToTicks(Constants.TURRET_TURN_DAMPER * VisionData.getXAngle());
      relativeMoveTurretPID(ticks);
    }
  }

  /**
   * Adds angle to current setpoint
   * @param setpoint inputs position to move to (in ticks)
   */
  public void relativeMoveTurretPID(double setpoint) {
    // Normalise setpoint
    if (Turret.motor.getSelectedSensorPosition() + setpoint > Turret.config.getUpperLimit()) setpoint = 0;
    else if (Turret.motor.getSelectedSensorPosition() + setpoint < Turret.config.getLowerLimit()) setpoint = 0;

    // Move turret toward setpoint
    moveTurretPID(Turret.motor.getClosedLoopTarget() + setpoint);
  }

  /**
   * Checks if turret PID motion is complete
   * @return True if motion is completed else false
   */
  public boolean turretCheckIfMotionComplete() {
    return Math.abs(Turret.motor.getSelectedSensorPosition() - Turret.motor.getClosedLoopTarget()) < Turret.config.getTolerance();
  }

  /**
   * Get the angle of the turret relative to the robot
   * @return current angle of the turret in degrees
   */
  public double getTurretAngle() {
    return Turret.ticksToDegrees(Turret.motor.getSelectedSensorPosition());
  }

  /**
   * Moves flywheel to a speed
   * @param speed input speed to keep the motor at (RPM)
   */
  public void setFlywheelSpeed(double speed) {
    speed = MathUtil.clamp(speed, 0, Flywheel.MAX_SPEED_RPM);
    double speedInTicks = Flywheel.rpmToTicksPer100ms(speed);

    // PID controller on Talon uses 1023 as "full output" 
    Flywheel.masterMotor.set(ControlMode.Velocity, speedInTicks);
  }

  /**
   * Move flywheel at specified speed
   * @param speed flywheel speed [-1, 1]
   */
  public void flywheelManual(double speed) {
    Flywheel.masterMotor.set(speed);
  }

  /**
   * Stop flywheel motor
   */
  public void flywheelStop() {
    Flywheel.masterMotor.set(0);
    Flywheel.masterMotor.setIntegralAccumulator(0);
  }

  /**
   * Checks if flywheel is at set speed
   * @return True if flywheel is at speed else false
   */
  public boolean isFlywheelAtSpeed() {
    boolean atSpeed = (Math.abs(flywheelError()) < Flywheel.masterConfig.getTolerance())
                      && Flywheel.masterMotor.getClosedLoopTarget() != 0;
    return atSpeed;
  }

  /**
   * Get error in flywheel speed
   * @return flywheel error (ticks per 100ms)
   */
  public double flywheelError() {
    return Flywheel.masterMotor.getClosedLoopTarget() - Flywheel.masterMotor.getSelectedSensorVelocity();
  }

  /**
   * Reset encoder to 0 to keep it in sync
   * @param motor resets input encoder
   */
  public void resetEncoder(BaseTalon motor) {
    motor.setSelectedSensorPosition(0);
  }

  /**
   * @return if the front limit switch is pressed
   */
  public boolean turretLimitFront() {
    return Turret.motor.getSensorCollection().isRevLimitSwitchClosed();
  }

  /**
   * @return if the back limit switch is pressed
   */
  public boolean turretLimitBack() {
    return Turret.motor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public void toggleFlywheel(double speed) {
    if (!Flywheel.isRunning) {
      flywheelManual(speed);
      Flywheel.isRunning = true;
    }
    else {
      flywheelStop();
      Flywheel.isRunning = false;
    }
  }

  /**
   * @return if the back limit switch is pressed
   */
  public boolean hoodLimit() {
    return Hood.motor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Reset the turret encoder to the front position
    if (turretLimitFront() && Turret.needsReset) {
      Turret.needsReset = false;
      Turret.motor.setSelectedSensorPosition(Constants.TURRET_FRONT_LIMIT_POSITION);
    } else if (!turretLimitFront() && !turretLimitBack()) Turret.needsReset = true;

    // Reset the turret encoder to the back position
    if (turretLimitBack() && Turret.needsReset) {
      Turret.needsReset = false;
      Turret.motor.setSelectedSensorPosition(Constants.TURRET_BACK_LIMIT_POSITION);
    } else if (!turretLimitBack() && !turretLimitFront()) Turret.needsReset = true;

    // Reset the hood encoder to the back position
    if (hoodLimit() && Hood.needsReset) {
      Hood.needsReset = false;
      Hood.motor.setSelectedSensorPosition(Constants.HOOD_BOTTOM_POSITION);
    } else if (!hoodLimit()) Hood.needsReset = true;
  }

  @Override
  public void close() {
    Flywheel.masterMotor = null;
    Flywheel.slaveMotor = null;
    Hood.motor = null;
    Turret.motor = null;
  }
}
