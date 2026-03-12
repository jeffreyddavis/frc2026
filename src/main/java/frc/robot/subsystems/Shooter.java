package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final boolean hardwareEnabled = Constants.Shooter.HardwareEnabled;

  private TalonFX leader;
  private TalonFX follower;

  // Control requests (reuse objects)
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final VelocityVoltage velocityFOC = new VelocityVoltage(0.0);

  // AdvantageKit tunables
  // private final LoggedNetworkNumber kP = new LoggedNetworkNumber("Shooter/kP", .001);

  // private final LoggedNetworkNumber kD = new LoggedNetworkNumber("Shooter/kD", 0.0);

  // private final LoggedNetworkNumber targetRPM =
  //    new LoggedNetworkNumber("Shooter/TargetRPM", 2000.0);

  @AutoLogOutput private double targetRPM = 0;

  // private final LoggedNetworkNumber kV = new LoggedNetworkNumber("Shooter/kV", 0.12);

  private static final double kP = 0.1;
  private static final double kD = 0.0;
  private static final double kV = 0.1159;

  // State
  private boolean closedLoop = true;
  private double lastTargetRPS = 0.0;

  private double timeWithinTolerance = 0.0;
  private double lastTimestamp = 0.0;

  private static final double STABLE_TIME = 0.10; // seconds

  // Config reused
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public Shooter() {

    if (hardwareEnabled) {
      leader = new TalonFX(Constants.Shooter.LeftMotor);
      follower = new TalonFX(Constants.Shooter.RightMotor);

      // Basic config
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 45;

      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 110;

      // Initial gains
      config.Slot0.kP = kP;
      config.Slot0.kD = kD;
      config.Slot0.kV = kV;
      config.Slot0.kI = 0.0;
      velocityFOC.withSlot(0);
      velocityFOC.withEnableFOC(true);

      leader.getConfigurator().apply(config);
      follower.getConfigurator().apply(config);

      // Follower
      follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

      // Faster status updates
      leader.getVelocity().setUpdateFrequency(100);
      leader.getStatorCurrent().setUpdateFrequency(100);
      leader.getSupplyCurrent().setUpdateFrequency(100);
      leader.getMotorVoltage().setUpdateFrequency(100);
      leader.optimizeBusUtilization();
    }
  }

  @Override
  public void periodic() {

    if (hardwareEnabled && closedLoop) {
      double rpm = targetRPM;
      double rps = rpm / 60.0;
      lastTargetRPS = rps;

      leader.setControl(velocityFOC.withVelocity(rps));
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      double dt = now - lastTimestamp;
      lastTimestamp = now;

      double velocityRPS = hardwareEnabled ? leader.getVelocity().getValueAsDouble() : 0.0;

      double error = Math.abs(velocityRPS - lastTargetRPS);

      if (error < Constants.Shooter.RPSTolerance) {
        timeWithinTolerance += dt;
      } else {
        timeWithinTolerance = 0.0;
      }
    }

    logTelemetry();
  }

  private void logTelemetry() {

    double velocityRPM = 0.0;
    double statorCurrent = 0.0;
    double supplyCurrent = 0.0;
    double appliedVolts = 0.0;

    if (hardwareEnabled) {
      double velocityRPS = leader.getVelocity().getValueAsDouble();
      velocityRPM = velocityRPS * 60.0;
      statorCurrent = leader.getStatorCurrent().getValueAsDouble();
      supplyCurrent = leader.getSupplyCurrent().getValueAsDouble();
      appliedVolts = leader.getMotorVoltage().getValueAsDouble();
      Logger.recordOutput(
          "Shooter/RPMError", (leader.getVelocity().getValueAsDouble() - lastTargetRPS) * 60.0);
    }

    Logger.recordOutput("Shooter/VelocityRPM", velocityRPM);
    Logger.recordOutput("Shooter/StatorCurrent", statorCurrent);
    Logger.recordOutput("Shooter/SupplyCurrent", supplyCurrent);
    Logger.recordOutput("Shooter/AppliedVolts", appliedVolts);
    Logger.recordOutput("Shooter/ClosedLoop", closedLoop);
    Logger.recordOutput("Shooter/HardwareEnabled", hardwareEnabled);
    Logger.recordOutput("Shooter/StableTime", timeWithinTolerance);
  }

  // =====================
  // Public Control Methods
  // =====================

  public void setOpenLoopVolts(double volts) {
    closedLoop = false;
    lastTargetRPS = 0.0;

    if (hardwareEnabled) {
      leader.setControl(voltageOut.withOutput(volts));
    }
  }

  public void jogPercent(double percent) {
    closedLoop = false;

    if (hardwareEnabled) {

      double appliedVolts = leader.getMotorVoltage().getValueAsDouble();
      double oldpercent = appliedVolts / 12.0;
      double newPercent = oldpercent + percent;
      if (newPercent == 0) newPercent = Math.signum(percent) * .01;
      leader.setControl(voltageOut.withOutput(newPercent * 12.0));
    }
  }

  public void enableClosedLoop() {
    closedLoop = true;
  }

  public void disable() {
    closedLoop = false;

    if (hardwareEnabled) {
      leader.setControl(voltageOut.withOutput(0.0));
    }
  }

  public void setTargetRPM(double rpm) {

    // Update NT target so it shows in AdvantageScope
    targetRPM = rpm;

    closedLoop = true;

    // Update cached target immediately for logging and tolerance checks
    lastTargetRPS = rpm / 60.0;
    timeWithinTolerance = 0.0; // reset stability
  }

  public double getVelocityRPM() {
    if (!hardwareEnabled) return 0.0;
    return leader.getVelocity().getValueAsDouble() * 60.0;
  }

  public boolean isAtSetpoint() {
    if (!hardwareEnabled) return false;
    return timeWithinTolerance > STABLE_TIME;
  }
}
