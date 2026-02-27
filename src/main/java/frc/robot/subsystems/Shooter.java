package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {

  private final TalonFX leader;
  private final TalonFX follower;

  // Control requests (reuse objects)
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final VelocityTorqueCurrentFOC velocityFOC =
      new VelocityTorqueCurrentFOC(0.0);

  // AdvantageKit tunables
  private final LoggedNetworkNumber kP =
      new LoggedNetworkNumber("Shooter/kP", 0.1);

  private final LoggedNetworkNumber kD =
      new LoggedNetworkNumber("Shooter/kD", 0.0);

  private final LoggedNetworkNumber targetRPM =
      new LoggedNetworkNumber("Shooter/TargetRPM", 2000.0);

  // Gain tracking
  private double lastKP;
  private double lastKD;

  // State
  private boolean closedLoop = false;
  private double lastTargetRPS = 0.0;

  // Config object reused
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public Shooter(int leaderId, int followerId) {
    leader = new TalonFX(leaderId);
    follower = new TalonFX(followerId);

    // Basic config
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 45;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 110;

    // Initial gains
    config.Slot0.kP = kP.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kI = 0.0;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    // Follower
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    // Faster velocity updates for logging
    leader.getVelocity().setUpdateFrequency(100);
    leader.getStatorCurrent().setUpdateFrequency(100);
    leader.getSupplyCurrent().setUpdateFrequency(100);
    leader.getMotorVoltage().setUpdateFrequency(100);
    leader.optimizeBusUtilization();

    lastKP = kP.get();
    lastKD = kD.get();
  }

  @Override
  public void periodic() {

    updateGainsIfChanged();

    if (closedLoop) {
      double rpm = targetRPM.get();
      double rps = rpm / 60.0;
      lastTargetRPS = rps;
      leader.setControl(velocityFOC.withVelocity(rps));
    }

    logTelemetry();
  }

  private void updateGainsIfChanged() {
    double currentKP = kP.get();
    double currentKD = kD.get();

    if (Math.abs(currentKP - lastKP) > 1e-6 ||
        Math.abs(currentKD - lastKD) > 1e-6) {

      config.Slot0.kP = currentKP;
      config.Slot0.kD = currentKD;
      config.Slot0.kI = 0.0;

      leader.getConfigurator().apply(config);

      lastKP = currentKP;
      lastKD = currentKD;
    }
  }

  private void logTelemetry() {
    double velocityRPS = leader.getVelocity().getValueAsDouble();
    double velocityRPM = velocityRPS * 60.0;

    Logger.recordOutput("Shooter/VelocityRPM", velocityRPM);
    Logger.recordOutput("Shooter/TargetRPM", lastTargetRPS * 60.0);
    Logger.recordOutput("Shooter/StatorCurrent",
        leader.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/SupplyCurrent",
        leader.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/AppliedVolts",
        leader.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Shooter/ClosedLoop", closedLoop);
  }

  // =====================
  // Public Control Methods
  // =====================

  public void setOpenLoopVolts(double volts) {
    closedLoop = false;
    lastTargetRPS = 0.0;
    leader.setControl(voltageOut.withOutput(volts));
  }

  public void enableClosedLoop() {
    closedLoop = true;
  }

  public void disable() {
    closedLoop = false;
    leader.setControl(voltageOut.withOutput(0.0));
  }

  public double getVelocityRPM() {
    return leader.getVelocity().getValueAsDouble() * 60.0;
  }
}