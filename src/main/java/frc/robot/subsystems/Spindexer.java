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

  private final TalonFX motor = new TalonFX(Constants.Spindexer.Motor);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  /* ===================== Tunables ===================== */

  private final LoggedNetworkNumber feedPercent =
      new LoggedNetworkNumber("Spindexer/FeedPercent", 0.5);

  private final LoggedNetworkNumber reversePercent =
      new LoggedNetworkNumber("Spindexer/ReversePercent", -0.4);

  private final LoggedNetworkNumber holdPercent =
      new LoggedNetworkNumber("Spindexer/HoldPercent", 0.1);

  private final LoggedNetworkNumber supplyCurrentLimit =
      new LoggedNetworkNumber("Spindexer/SupplyCurrentLimit", 35.0);

  /* ===================== State ===================== */

  private double commandedPercent = 0.0;
  private boolean running = false;

  public Spindexer() {
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(baseConfig);

    CurrentLimitsConfigs limits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyCurrentLimit.get());

    motor.getConfigurator().apply(limits);

    motor.getVelocity().setUpdateFrequency(100);
    motor.getSupplyCurrent().setUpdateFrequency(100);
    motor.getStatorCurrent().setUpdateFrequency(100);
    motor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
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
    voltageOut.Output = percent * 12.0;
    motor.setControl(voltageOut);
  }

  /* ===================== Logging ===================== */

  private void logTelemetry() {
    Logger.recordOutput("Spindexer/VelocityRPS", motor.getVelocity().getValueAsDouble());

    Logger.recordOutput("Spindexer/VelocityRPM", motor.getVelocity().getValueAsDouble() * 60.0);

    Logger.recordOutput("Spindexer/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());

    Logger.recordOutput("Spindexer/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());

    Logger.recordOutput("Spindexer/CommandedPercent", commandedPercent);

    Logger.recordOutput("Spindexer/Running", running);
  }
}
