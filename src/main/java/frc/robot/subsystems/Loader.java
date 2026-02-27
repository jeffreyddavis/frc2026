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

  private final TalonFX motor = new TalonFX(Constants.Loader.Motor);

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

  public Loader() {
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);

    // Current limits are applied separately
    CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40.0)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(100.0);

    motor.getConfigurator().apply(currentLimits);
  }

  private double lastSupplyLimit = currentLimit.get();

  @Override
  public void periodic() {

    double newLimit = currentLimit.get();

    if (Math.abs(newLimit - lastSupplyLimit) > 1e-6) {

      CurrentLimitsConfigs limits =
          new CurrentLimitsConfigs()
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentLimit(newLimit);

      motor.getConfigurator().apply(limits);

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
    voltageOut.Output = percent * 12.0;
    motor.setControl(voltageOut);
  }

  /* ===================== Logging ===================== */

  private void logTelemetry() {
    Logger.recordOutput("Loader/VelocityRPS", motor.getVelocity().getValueAsDouble());

    Logger.recordOutput("Loader/VelocityRPM", motor.getVelocity().getValueAsDouble() * 60.0);

    Logger.recordOutput("Loader/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());

    Logger.recordOutput("Loader/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());

    Logger.recordOutput("Loader/CommandedPercent", commandedPercent);

    Logger.recordOutput("Loader/Running", running);
  }
}
