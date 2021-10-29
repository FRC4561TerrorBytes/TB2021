package frc.robot.subsystems;

import static org.mockito.Mockito.*;
import org.mockito.*;
import org.junit.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

public class DriveSubsystemTest {
  public static final double DELTA = 1e-2;
  private DifferentialDrive m_drivetrain;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;

  private WPI_TalonFX m_leftMasterMotor;
  private WPI_TalonFX m_rightMasterMotor;
  private WPI_TalonFX m_leftSlaveMotor;
  private WPI_TalonFX m_rightSlaveMotor;

  private Counter m_lidar;
  private AHRS m_navx;

  @Before
  public void setup() {
    assert HAL.initialize(500, 0);
    m_leftMasterMotor = mock(WPI_TalonFX.class);
    m_rightMasterMotor = mock(WPI_TalonFX.class);
    m_leftSlaveMotor = mock(WPI_TalonFX.class);
    m_rightSlaveMotor = mock(WPI_TalonFX.class);
    m_lidar = mock(Counter.class);
    m_navx = mock(AHRS.class);

    m_drivetrain = mock(DifferentialDrive.class);

    m_drivetrainHardware = new DriveSubsystem.Hardware(m_drivetrain, m_leftMasterMotor, m_rightMasterMotor, m_leftSlaveMotor, m_rightSlaveMotor, m_lidar, m_navx);

    m_driveSubsystem = new DriveSubsystem(m_drivetrainHardware, 
                                          Constants.DRIVE_kP,
                                          Constants.DRIVE_kD, 
                                          Constants.DRIVE_TOLERANCE,
                                          Constants.DRIVE_TURN_SCALAR,
                                          Constants.DEADBAND,
                                          Constants.DRIVE_TRACTION_CONTROL_CURVE);
  }

  @After
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
  }

  @Test
  public void goingForward() {
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityX()).thenReturn((float)3.966);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    for (int i = 0; i < 60; i++) { m_driveSubsystem.periodic(); }
    m_driveSubsystem.teleopPID(1.0, 0.0, 1);

    verify(m_drivetrain, times(1)).arcadeDrive(AdditionalMatchers.eq(1.0, DELTA), AdditionalMatchers.eq(0.0, 0.0), eq(false));
  }

  @Test
  public void turningLeft() {
    when(m_navx.getAngle()).thenReturn(0.0);

    m_driveSubsystem.teleopPID(0.0, -1.0, 1);

    verify(m_drivetrain, times(1)).arcadeDrive(AdditionalMatchers.eq(0.0, 0.0), AdditionalMatchers.gt(0.0), eq(false));
  }

  @Test
  public void turningRight() {
    when(m_navx.getAngle()).thenReturn(0.0);

    m_driveSubsystem.teleopPID(0.0, 1.0, 1);

    verify(m_drivetrain, times(1)).arcadeDrive(AdditionalMatchers.eq(0.0, 0.0), AdditionalMatchers.lt(0.0), eq(false));
  }

  @Test
  public void tractionControl() {
    when(m_navx.getVelocityX()).thenReturn((float)0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    for (int i = 0; i < 60; i++) { m_driveSubsystem.periodic(); }
    m_driveSubsystem.teleopPID(1.0, 0.0, 1);

    verify(m_drivetrain, times(1)).arcadeDrive(AdditionalMatchers.eq(0.03, DELTA), AdditionalMatchers.eq(0.0, 0.0), eq(false));
  }
}
