package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
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
      new LoggedNetworkNumber("Spindexer/FeedPercent", 0.2);

  private final LoggedNetworkNumber reversePercent =
      new LoggedNetworkNumber("Spindexer/ReversePercent", -0.4);

  private final LoggedNetworkNumber holdPercent =
      new LoggedNetworkNumber("Spindexer/HoldPercent", 0.1);

  private final LoggedNetworkNumber supplyCurrentLimit =
      new LoggedNetworkNumber("Spindexer/SupplyCurrentLimit", 35.0);

  private final LoggedNetworkNumber unjamIntervalSec =
      new LoggedNetworkNumber("Spindexer/UnjamIntervalSec", 2.0);

  private final LoggedNetworkNumber unjamDurationSec =
      new LoggedNetworkNumber("Spindexer/UnjamDurationSec", 0.15);

  private final LoggedNetworkNumber unjamReversePercent =
      new LoggedNetworkNumber("Spindexer/UnjamReversePercent", -0.6);

  /* ===================== State ===================== */

  private double commandedPercent = 0.0;
  private boolean running = false;
  private double lastAppliedCurrentLimit = 0.0;

  private enum FeedState {
    FORWARD,
    UNJAM
  }

  private FeedState feedState = FeedState.FORWARD;
  private double stateStartTime = 0.0;

  // Track transitions
  private boolean feedingActive = false;

  // Timing
  private double cycleStartTime = 0.0;

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

    double now = Timer.getFPGATimestamp();

    // Live-update current limit
    double newLimit = supplyCurrentLimit.get();
    if (hardwareEnabled && Math.abs(newLimit - lastAppliedCurrentLimit) > 1e-6) {
      applyCurrentLimit(newLimit);
      lastAppliedCurrentLimit = newLimit;
    }

    // ===================== FEED STATE MACHINE =====================
    if (running && feedingActive) {

      double elapsed = now - cycleStartTime;

      switch (feedState) {
        case FORWARD:
          applyPercent(feedPercent.get());

          // Only transition AFTER interval has passed
          if (elapsed > unjamIntervalSec.get()) {
            feedState = FeedState.UNJAM;
            cycleStartTime = now;
          }
          break;

        case UNJAM:
          applyPercent(unjamReversePercent.get());

          if (elapsed > unjamDurationSec.get()) {
            feedState = FeedState.FORWARD;

            // Restart interval AFTER unjam completes
            cycleStartTime = now;
          }
          break;
      }

    } else if (running) {
      // reverse(), hold(), manual → behave normally
      applyPercent(commandedPercent);
    } else {
      applyPercent(0.0);
    }

    logTelemetry();
  }

  /* ===================== Public Control ===================== */

  public void setFeedPercent(double percent) {
    feedPercent.set(percent);
  }

  public void feed() {
    commandedPercent = feedPercent.get();

    // Only trigger on transition into feeding
    if (!feedingActive) {
      feedingActive = true;

      feedState = FeedState.FORWARD;

      // IMPORTANT: start cycle NOW so we don’t immediately unjam
      cycleStartTime = Timer.getFPGATimestamp();
    }

    running = true;
  }

  public void reverse() {
    running = true;
    commandedPercent = reversePercent.get();
  }

  public void hold() {
    running = true;
    commandedPercent = holdPercent.get();
  }

  public void stop() {
    running = false;
    feedingActive = false;
    commandedPercent = 0.0;
  }

  public void setManualPercent(double percent) {
    running = true;
    commandedPercent = percent;
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
