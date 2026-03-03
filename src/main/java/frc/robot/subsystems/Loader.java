package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Loader extends SubsystemBase {

  private final boolean hardwareEnabled = Constants.Loader.HardwareEnabled;

  private TalonFX motor;

  private final VoltageOut voltageOut = new VoltageOut(0.0);

  /* ===================== Tunables ===================== */

  private final LoggedNetworkNumber feedPercent =
      new LoggedNetworkNumber("Loader/FeedPercent", 0.5);

  private final LoggedNetworkNumber reversePercent =
      new LoggedNetworkNumber("Loader/ReversePercent", -0.3);

  private final LoggedNetworkNumber currentLimit =
      new LoggedNetworkNumber("Loader/SupplyCurrentLimit", 40.0);

  /* ===================== State ===================== */

  private boolean running = false;
  private double commandedPercent = 0.0;
  private double lastSupplyLimit = currentLimit.get();

  public Loader() {

    if (hardwareEnabled) {
      motor = new TalonFX(Constants.Loader.Motor);
      configureMotor();
      lastSupplyLimit = currentLimit.get();
    }
  }

  private void configureMotor() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);

    applyCurrentLimit(currentLimit.get());

    motor.getVelocity().setUpdateFrequency(100);
    motor.getSupplyCurrent().setUpdateFrequency(100);
    motor.getStatorCurrent().setUpdateFrequency(100);
    motor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {

    // Live-update supply current limit
    double newLimit = currentLimit.get();

    if (hardwareEnabled && Math.abs(newLimit - lastSupplyLimit) > 1e-6) {

      applyCurrentLimit(newLimit);
      lastSupplyLimit = newLimit;
    }

    logTelemetry();
  }

  /* ===================== Public Control ===================== */

  public void feed() {
    running = true;
    commandedPercent = feedPercent.get();
    setPercentOutput(commandedPercent);
  }

  public void reverse() {
    running = true;
    commandedPercent = reversePercent.get();
    setPercentOutput(commandedPercent);
  }

  public void stop() {
    running = false;
    commandedPercent = 0.0;
    setPercentOutput(0.0);
  }

  public void setManualPercent(double percent) {
    running = true;
    commandedPercent = percent;
    setPercentOutput(percent);
  }

  private void setPercentOutput(double percent) {
    if (!hardwareEnabled) return;

    voltageOut.Output = percent * 12.0;
    motor.setControl(voltageOut);
  }

  private void applyCurrentLimit(double limit) {
    CurrentLimitsConfigs limits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(limit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(100.0);

    motor.getConfigurator().apply(limits);
  }

  /* ===================== Logging ===================== */

  private void logTelemetry() {

    double velocityRPS = 0.0;
    double supplyCurrent = 0.0;
    double statorCurrent = 0.0;

    if (hardwareEnabled) {
      velocityRPS = motor.getVelocity().getValueAsDouble();
      supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
      statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    }

    Logger.recordOutput("Loader/VelocityRPS", velocityRPS);
    Logger.recordOutput("Loader/VelocityRPM", velocityRPS * 60.0);
    Logger.recordOutput("Loader/SupplyCurrent", supplyCurrent);
    Logger.recordOutput("Loader/StatorCurrent", statorCurrent);
    Logger.recordOutput("Loader/CommandedPercent", commandedPercent);
    Logger.recordOutput("Loader/Running", running);
    Logger.recordOutput("Loader/HardwareEnabled", hardwareEnabled);
  }
}
