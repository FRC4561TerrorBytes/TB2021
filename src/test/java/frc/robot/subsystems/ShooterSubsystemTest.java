package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import frc.robot.Constants;

public class ShooterSubsystemTest {
  public static final double DELTA = 1e-3;
  private ShooterSubsystem m_shooterSubsystem;
  private ShooterSubsystem.Hardware m_shooterHardware;
  private WPI_TalonFX m_flywheelMasterMotor;
  private WPI_TalonFX m_flywheelSlaveMotor;
  private WPI_TalonSRX m_hoodMotor;
  private WPI_TalonSRX m_turretMotor;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_flywheelMasterMotor = mock(WPI_TalonFX.class);
    m_flywheelSlaveMotor = mock(WPI_TalonFX.class);
    m_hoodMotor = mock(WPI_TalonSRX.class);
    m_turretMotor = mock(WPI_TalonSRX.class);

    // Create Hardware object using mock objects
    m_shooterHardware = new ShooterSubsystem.Hardware(m_flywheelMasterMotor, m_flywheelSlaveMotor, m_hoodMotor, m_turretMotor);

    // Create ShooterSubsystem object
    m_shooterSubsystem = new ShooterSubsystem(m_shooterHardware,
                                              Constants.FLYWHEEL_MASTER_CONFIG,
                                              Constants.HOOD_CONFIG,
                                              Constants.TURRET_CONFIG);
  }

  @AfterEach
  public void close() {
    m_shooterSubsystem.close();
    m_shooterSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can spin up flywheel")
  public void spinFlywheel() {
    m_shooterSubsystem.setFlywheelSpeed(5400);

    verify(m_flywheelMasterMotor, times(1)).set(ArgumentMatchers.eq(ControlMode.Velocity), 
                                                AdditionalMatchers.eq(18432, DELTA), 
                                                ArgumentMatchers.eq(DemandType.ArbitraryFeedForward),
                                                AdditionalMatchers.eq(1, DELTA));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can stop flywheel")
  public void stopFlywheel() {
    m_shooterSubsystem.flywheelStop();
    
    verify(m_flywheelMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA));
    assertEquals(0.0, m_flywheelMasterMotor.getIntegralAccumulator(), DELTA);
  }
  
}
