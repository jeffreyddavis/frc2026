package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {

  private final boolean hardwareEnabled = Constants.Turret.HardwareEnabled;

  private TalonFX motor;
  private CANcoder encoder;

  private final VoltageOut voltageOut = new VoltageOut(0.0);

  /* ===================== Tunables ===================== */

  private final LoggedNetworkNumber targetAngleDeg =
      new LoggedNetworkNumber("Turret/TargetDeg", 0.0);

  private final LoggedNetworkNumber lookAheadSeconds =
      new LoggedNetworkNumber("Turret/lookAheadSeconds", 0.0);

  // private final LoggedNetworkNumber kP = new LoggedNetworkNumber("Turret/kP", 0.00002);

  // private final LoggedNetworkNumber kSVolts = new LoggedNetworkNumber("Turret/kSVolts", 0.0);

  // private final LoggedNetworkNumber maxOutput = new LoggedNetworkNumber("Turret/MaxOutput", 0.4);

  // private final LoggedNetworkNumber toleranceDeg =
  //   new LoggedNetworkNumber("Turret/ToleranceDeg", 2.0);

  private double robotOmegaDegPerSec = 0;

  /* ===================== State ===================== */

  private boolean closedLoop = true;

  public Turret() {

    if (hardwareEnabled) {
      motor = new TalonFX(Constants.Turret.Motor);
      encoder = new CANcoder(Constants.Turret.Encoder);

      configureMotor();
      configureEncoder();
      zeroTurret(); // Use boot position as zero
    }
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 25;

    motor.getConfigurator().apply(config);
  }

  private void configureEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = Constants.Turret.EncoderOffset;
    encoder.getConfigurator().apply(config);
  }

  /* ===================== Zeroing ===================== */

  public void zeroTurret() {
    if (!hardwareEnabled) return;
    encoder.setPosition(0.0); // rotations = 0 at boot
  }

  /* ===================== Public Control ===================== */

  public void enableClosedLoop() {
    closedLoop = true;
  }

  public void disable() {
    closedLoop = false;

    if (hardwareEnabled) {
      setPercentOutput(0);
    }
  }

  public void setManualPercent(double percent) {
    closedLoop = false;

    if (hardwareEnabled) {
      setPercentOutput(percent);
    }
  }

  public boolean isAtSetpoint() {
    if (!hardwareEnabled) return false;

    double current = getTurretAngleDegrees();
    double target = targetAngleDeg.get();
    return Math.abs(current - target) < Constants.Turret.toleranceDeg; // toleranceDeg.get();
  }

  /* ===================== Core Logic ===================== */

  @Override
  public void periodic() {

    double current = hardwareEnabled ? getTurretAngleDegrees() : 0.0;

    double target = targetAngleDeg.get();

    // prediction horizon (seconds)
    double lookahead = Constants.Turret.OMEGA_LOOKAHEAD;

    // predict where the target will be
    double predictedTarget = target - (robotOmegaDegPerSec * lookahead);
    // double predictedTarget = target + (lookAheadSeconds.get() * robotOmegaDegPerSec);

    double delta = computeSafeDelta(current, predictedTarget);

    Logger.recordOutput("Turret/CurrentDeg", current);
    Logger.recordOutput("Turret/TargetDeg", target);
    Logger.recordOutput("Turret/DeltaDeg", delta);
    Logger.recordOutput("Turret/ClosedLoop", closedLoop);
    Logger.recordOutput("Turret/HardwareEnabled", hardwareEnabled);
    Logger.recordOutput("Turret/RobotOmegaDegPerSec", robotOmegaDegPerSec);
    Logger.recordOutput("Turret/predictedTarget", predictedTarget);

    if (!closedLoop) return;

    if (Math.abs(delta) < Constants.Turret.toleranceDeg /*toleranceDeg.get()*/) {
      if (hardwareEnabled) {
        setPercentOutput(0);
      }
      Logger.recordOutput("Turret/Output", 0.0);
      return;
    }

    double output = (delta * Constants.Turret.kP);

    // kP.get();

    double max = Constants.Turret.maxOutput; // maxOutput.get();
    output = MathUtil.clamp(output, -max, max);

    if (hardwareEnabled) {
      setPercentOutput(-output); // direction is inverted
    }

    Logger.recordOutput("Turret/Output", output);
  }

  private void setPercentOutput(double percent) {

    double desiredOut = percent * 12.0;

    if (Math.abs(percent) > 1e-4) {
      desiredOut += Math.signum(percent) * Constants.Turret.kS; // kSVolts.get();
    }

    voltageOut.Output = desiredOut;
    motor.setControl(voltageOut);
  }

  public void setFieldTargetAngle(
      double fieldTargetDeg, Rotation2d robotHeading, double robotAngularVelocityDegPerSec) {

    robotOmegaDegPerSec = robotAngularVelocityDegPerSec;
    // Convert robot heading to degrees
    double robotDeg = robotHeading.getDegrees();

    // Turret target relative to robot frame
    double turretRelativeTarget = robotDeg - fieldTargetDeg;

    // Normalize to signed range
    turretRelativeTarget = normalizeToSigned(turretRelativeTarget);

    targetAngleDeg.set(turretRelativeTarget);

    closedLoop = true;
  }

  /* ===================== Jog Helpers ===================== */

  public void jogLeft() {
    // setPercentOutput(.1);
    targetAngleDeg.set(targetAngleDeg.get() - 1);
  }

  public void jogRight() {
    // setPercentOutput(-.1);
    targetAngleDeg.set(targetAngleDeg.get() + 1);
  }

  public void stop() {
    setPercentOutput(0);
  }

  /* ===================== Encoder ===================== */

  public double getTurretAngleDegrees() {
    if (!hardwareEnabled) return 0.0;
    return normalizeToSigned(
        encoder.getPosition().getValueAsDouble() * (360.0 / Constants.Turret.GEAR_RATIO));
  }

  /* ===================== Safety & Math ===================== */

  private double normalizeToSigned(double degrees) {
    double angle = degrees % 360.0;
    if (angle >= 180) angle -= 360;
    if (angle < -180) angle += 360;
    return angle;
  }

  private double computeSafeDelta(double currentDeg, double targetDeg) {

    double current = normalizeToSigned(currentDeg);
    double target = normalizeToSigned(targetDeg);
    // 176 compared to -176

    double maxAllowed = 180 - Constants.Turret.FORBIDDEN_BUFFER_DEG;
    //
    target =
        MathUtil.clamp(target, Constants.Turret.TURRET_MIN_DEG, Constants.Turret.TURRET_MAX_DEG);
    // -174

    double delta = target - current;
    // -174 - 176

    // if (delta > 180) delta -= 360;
    // if (delta < -180) delta += 360;

    return delta;
  }
}
