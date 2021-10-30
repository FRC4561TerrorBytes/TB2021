package frc.robot.commands;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.subsystems.MagazineSubsystem;


public class IntakeCommandTest {
  public static final double DELTA = 5e-3;
  private static MagazineSubsystem m_magazineSubsystem;
  private static MagazineSubsystem.Hardware m_magazineHardware;
  private static IntakeCommand m_intakeCommand;
  private static CANSparkMax m_intakeMotor;
  private static WPI_TalonSRX m_armMotor;
  private static WPI_TalonSRX m_magazineMotor;

  @BeforeAll
  public static void setup() {
    HAL.initialize(500, 0);

    m_intakeMotor = mock(CANSparkMax.class);
    m_armMotor = mock(WPI_TalonSRX.class);
    m_magazineMotor = mock(WPI_TalonSRX.class);

    m_magazineHardware = new MagazineSubsystem.Hardware(m_intakeMotor, m_armMotor, m_magazineMotor);

    m_magazineSubsystem = new MagazineSubsystem(m_magazineHardware, Constants.ARM_CONFIG);

    m_intakeCommand = new IntakeCommand(m_magazineSubsystem, 1.0);
  }

  @AfterAll
  public static void close() {
    m_magazineSubsystem.close();
    m_magazineSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can intake balls")
  public void intakeBalls() {
    // Try to intake balls
    m_intakeCommand.execute();

    // Verify if arm and intake motors are set with expected values
    verify(m_intakeMotor, times(1)).set(AdditionalMatchers.eq(1.0, DELTA));
    verify(m_armMotor, times(1)).set(AdditionalMatchers.eq(0.05, DELTA));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot stops intaking balls")
  public void stopIntakingBalls() {
    // Try to stop intaking balls
    m_intakeCommand.end(true);

    // Verify if arm and intake motors are set with expected values
    verify(m_intakeMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA));
    verify(m_armMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA));
  }
}
