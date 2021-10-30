package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;

public class MagazineSubsystemTest {
  public static final double DELTA = 5e-3;
  private MagazineSubsystem m_magazineSubsystem;
  private MagazineSubsystem.Hardware m_magazineHardware;
  private CANSparkMax m_intakeMotor;
  private WPI_TalonSRX m_armMotor;
  private WPI_TalonSRX m_magazineMotor;

  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);

    m_intakeMotor = mock(CANSparkMax.class);
    m_armMotor = mock(WPI_TalonSRX.class);
    m_magazineMotor = mock(WPI_TalonSRX.class);

    m_magazineHardware = new MagazineSubsystem.Hardware(m_intakeMotor, m_armMotor, m_magazineMotor);

    m_magazineSubsystem = new MagazineSubsystem(m_magazineHardware, Constants.ARM_CONFIG);
  }

  @AfterEach
  public void close() {
    m_magazineSubsystem.close();
    m_magazineSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can intake")
  public void intake() {
    // Trey to intake
    m_magazineSubsystem.intakeMotorSpeed(1.0);

    // Verify if intake motor is being set with expected value
    verify(m_intakeMotor, times(1)).set(AdditionalMatchers.eq(1.0, DELTA));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can outtake")
  public void outtake() {
    // Try to outtake
    m_magazineSubsystem.intakeMotorSpeed(-1.0);

    // Verify if intake motor is being set with expected value
    verify(m_intakeMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can stop intake")
  public void intakeStop() {
    // Try to stop intake
    m_magazineSubsystem.intakeMotorSpeed(0.0);

    // Verify if intake motor is being set with expected value
    verify(m_intakeMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA));
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can raise arm")
  public void raiseArm() {
    // Try to raise arm
    m_magazineSubsystem.armManual(1.0);

    // Verify if arm motor is being set with expected value
    verify(m_armMotor, times(1)).set(AdditionalMatchers.eq(1.0, DELTA));
  }

  @Test
  @Order(5)
  @DisplayName("Test if robot can lower arm")
  public void lowerArm() {
    // Try to lower arm
    m_magazineSubsystem.armManual(-1.0);

    // Verify if arm motor is being set with expected value
    verify(m_armMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA));
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot can uptake")
  public void uptake() {
    // Try to uptake
    m_magazineSubsystem.ballUptake(1.0);

    // Verify if uptake motor is being set with expected value
    verify(m_magazineMotor, times(1)).set(AdditionalMatchers.eq(1.0, DELTA));
  }

  @Test
  @Order(7)
  @DisplayName("Test if robot can downtake")
  public void downtake() {
    // Try to downtake
    m_magazineSubsystem.ballUptake(-1.0);

    // Verify if uptake motor is being set with expected value
    verify(m_magazineMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA));
  }

  @Test
  @Order(8)
  @DisplayName("Test if robot can stop uptake")
  public void stopUptake() {
    // Try to stop uptake
    m_magazineSubsystem.ballUptakeStop();

    // Verify if uptake motor is being set with expected value
    verify(m_magazineMotor, times(1)).set(AdditionalMatchers.eq(0, DELTA));
  }
}
