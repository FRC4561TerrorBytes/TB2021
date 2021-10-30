package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
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
  @DisplayName("Test if robot can intake")
  public void intake() {
    m_magazineSubsystem.intakeMotorSpeed(1.0);
    verify(m_intakeMotor, times(1)).set(AdditionalMatchers.eq(1.0, DELTA));
  }

  @Test
  @DisplayName("Test if robot can outtake")
  public void outtake() {
    m_magazineSubsystem.intakeMotorSpeed(-1.0);
    verify(m_intakeMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA));
  }
}
