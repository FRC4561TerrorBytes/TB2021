package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import edu.wpi.first.wpilibj.Counter;
import frc.robot.Constants;

public class DriveSubsystemTest {
  public static final double DELTA = 2e-3;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;

  private WPI_TalonFX m_lMasterMotor;
  private WPI_TalonFX m_rMasterMotor;
  private WPI_TalonFX m_leftSlaveMotor;
  private WPI_TalonFX m_rightSlaveMotor;

  private Counter m_lidar;
  private AHRS m_navx;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_lMasterMotor = mock(WPI_TalonFX.class);
    m_rMasterMotor = mock(WPI_TalonFX.class);
    m_leftSlaveMotor = mock(WPI_TalonFX.class);
    m_rightSlaveMotor = mock(WPI_TalonFX.class);
    m_lidar = mock(Counter.class);
    m_navx = mock(AHRS.class);

    // Create Hardware object using mock objects
    m_drivetrainHardware = new DriveSubsystem.Hardware(m_lMasterMotor, m_rMasterMotor, m_leftSlaveMotor, m_rightSlaveMotor, m_lidar, m_navx);

    // Create DriveSubsystem object
    m_driveSubsystem = new DriveSubsystem(m_drivetrainHardware, 
                                          Constants.DRIVE_kP,
                                          Constants.DRIVE_kD, 
                                          Constants.DRIVE_TOLERANCE,
                                          Constants.DRIVE_TURN_SCALAR,
                                          Constants.DRIVE_TRACTION_CONTROL_CURVE);
  }

  @AfterEach
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can move forward using PID drive")
  public void forward() {
    // Hardcode NAVX sensor return values for angle, velocityX, and velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)4.106);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Fill up velocity moving average buffer by calling periodic
    for (int i = 0; i < 60; i++) { m_driveSubsystem.periodic(); }

    // Try to drive forward
    m_driveSubsystem.teleopPID(1.0, 0.0, 1);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can move in reverse using PID drive")
  public void reverse() {
    // Hardcode NAVX sensor return values for angle, velocityX, and velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)4.106);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Fill up velocity moving average buffer by calling periodic
    for (int i = 0; i < 60; i++) { m_driveSubsystem.periodic(); }

    // Try to drive in reverse
    m_driveSubsystem.teleopPID(-1.0, 0.0, 1);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(-1.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(-1.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can stop using PID drive")
  public void stop() {
    // Hardcode NAVX sensor return value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Try to stop
    m_driveSubsystem.teleopPID(0.0, 0.0, 1);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot ignores small turn input values under threshold")
  public void ignoreSmallTurnInput() {
    // Hardcode NAVX sensor return values for angle, velocityX, velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)4.106);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Fill up velocity moving average buffer by calling periodic
    for (int i = 0; i < 60; i++) { m_driveSubsystem.periodic(); }

    // Try to drive with small turn value
    m_driveSubsystem.teleopPID(1.0, 0.001, 1);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(1.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(5)
  @DisplayName("Test if robot can turn left using PID drive")
  public void turningLeft() {
    // Hardcode NAVX sensor return value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Try to turn left
    m_driveSubsystem.teleopPID(0.0, -1.0, 1);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.gt(0.0));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.lt(0.0));
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot can turn right using PID drive")
  public void turningRight() {
    // Hardcode NAVX sensor return value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Try to turn right
    m_driveSubsystem.teleopPID(0.0, 1.0, 1);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.lt(0.0));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.0, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.gt(0.0));
  }

  @Test
  @Order(7)
  @DisplayName("Test if robot will limit wheel slip")
  public void tractionControl() {
    // Hardcode NAVX sensor values for angle, velocityX, and velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Fill up velocity moving average buffer by calling periodic
    for (int i = 0; i < 60; i++) { m_driveSubsystem.periodic(); }

    // Try to move forward
    m_driveSubsystem.teleopPID(1.0, 0.0, 1);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.03, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
    verify(m_rMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.PercentOutput), AdditionalMatchers.eq(0.03, DELTA), 
                                          ArgumentMatchers.eq(DemandType.ArbitraryFeedForward), AdditionalMatchers.eq(0.0, DELTA));
  }
}
