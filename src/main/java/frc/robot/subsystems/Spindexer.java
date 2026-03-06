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

public class Spindexer extends SubsystemBase {

  private final boolean hardwareEnabled = Constants.Spindexer.HardwareEnabled;

  private TalonFX motor;
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  /* ===================== Tunables ===================== */

  private final LoggedNetworkNumber feedPercent =
      new LoggedNetworkNumber("Spindexer/FeedPercent", 0.0);

  private final LoggedNetworkNumber reversePercent =
      new LoggedNetworkNumber("Spindexer/ReversePercent", -0.4);

  private final LoggedNetworkNumber holdPercent =
      new LoggedNetworkNumber("Spindexer/HoldPercent", 0.1);

  private final LoggedNetworkNumber supplyCurrentLimit =
      new LoggedNetworkNumber("Spindexer/SupplyCurrentLimit", 35.0);

  /* ===================== State ===================== */

  private double commandedPercent = 0.0;
  private boolean running = false;
  private double lastAppliedCurrentLimit = 0.0;

  public Spindexer() {

    if (hardwareEnabled) {
      motor = new TalonFX(Constants.Spindexer.Motor);
      configureMotor();
      lastAppliedCurrentLimit = supplyCurrentLimit.get();
    }
  }

  private void configureMotor() {
    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(baseConfig);

    applyCurrentLimit(supplyCurrentLimit.get());

    motor.getVelocity().setUpdateFrequency(100);
    motor.getSupplyCurrent().setUpdateFrequency(100);
    motor.getStatorCurrent().setUpdateFrequency(100);
    motor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {

    // Live-update current limit if changed
    double newLimit = supplyCurrentLimit.get();
    if (hardwareEnabled && Math.abs(newLimit - lastAppliedCurrentLimit) > 1e-6) {

      applyCurrentLimit(newLimit);
      lastAppliedCurrentLimit = newLimit;
    }

    logTelemetry();
  }

  /* ===================== Public Control ===================== */

  public void feed() {
    running = true;
    commandedPercent = feedPercent.get();
    applyPercent(commandedPercent);
  }

  public void reverse() {
    running = true;
    commandedPercent = reversePercent.get();
    applyPercent(commandedPercent);
  }

  public void hold() {
    running = true;
    commandedPercent = holdPercent.get();
    applyPercent(commandedPercent);
  }

  public void stop() {
    running = false;
    commandedPercent = 0.0;
    applyPercent(0.0);
  }

  public void setManualPercent(double percent) {
    running = true;
    commandedPercent = percent;
    applyPercent(percent);
  }

  private void applyPercent(double percent) {
    if (!hardwareEnabled) return;

    voltageOut.Output = percent * 12.0;
    motor.setControl(voltageOut);
  }

  private void applyCurrentLimit(double limit) {
    CurrentLimitsConfigs limits =
        new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(limit);

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

    Logger.recordOutput("Spindexer/VelocityRPS", velocityRPS);
    Logger.recordOutput("Spindexer/VelocityRPM", velocityRPS * 60.0);
    Logger.recordOutput("Spindexer/SupplyCurrent", supplyCurrent);
    Logger.recordOutput("Spindexer/StatorCurrent", statorCurrent);
    Logger.recordOutput("Spindexer/CommandedPercent", commandedPercent);
    Logger.recordOutput("Spindexer/Running", running);
    Logger.recordOutput("Spindexer/HardwareEnabled", hardwareEnabled);
  }
}
