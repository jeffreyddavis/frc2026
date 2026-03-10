package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {

  private final boolean hardwareEnabled = Constants.Intake.HardwareEnabled;

  /* ===================== Hardware ===================== */

  private TalonFX armLeader;
  private TalonFX armFollower;

  private CANcoder armEncoder;

  private final VoltageOut armVoltage = new VoltageOut(0.0);

  private static final double ARM_MIN = 2.0;
  private static final double ARM_MAX = 90;

  private SparkFlex rollerLeft;
  private SparkFlex rollerRight;

  /* ===================== Tunables ===================== */

  private final LoggedNetworkNumber deployPercent =
      new LoggedNetworkNumber("Intake/ArmDeployPercent", 0.0);

  private final LoggedNetworkNumber retractPercent =
      new LoggedNetworkNumber("Intake/ArmRetractPercent", -0.0);

  private final LoggedNetworkNumber armCurrentLimit =
      new LoggedNetworkNumber("Intake/ArmSupplyLimit", 40.0);

  private final LoggedNetworkNumber intakePercent =
      new LoggedNetworkNumber("Intake/RollerIntakePercent", 0.0);

  private final LoggedNetworkNumber outtakePercent =
      new LoggedNetworkNumber("Intake/RollerOuttakePercent", -0.1);

  private final LoggedNetworkNumber holdPercent =
      new LoggedNetworkNumber("Intake/RollerHoldPercent", 0.1);

  private final LoggedNetworkNumber arbitraryAngle =
      new LoggedNetworkNumber("Intake/arbitraryAngle", 45);

  // private final LoggedNetworkNumber kP = new LoggedNetworkNumber("Intake/kP", 20);
  private final double kP = .7;

  // private final LoggedNetworkNumber kI = new LoggedNetworkNumber("Intake/kI", .1);
  private final double kI = 0;

  // private final LoggedNetworkNumber kS = new LoggedNetworkNumber("Intake/kS", 1.7);
  private final double kS = 0;

  // private final LoggedNetworkNumber kG = new LoggedNetworkNumber("Intake/kG", 1);
  private final double kG = .01;

  /* ===================== State ===================== */

  private double armCommanded = 0.0;
  private double rollerCommanded = 0.0;
  private double lastArmLimit = 0.0;

  @AutoLogOutput private double armTargetDegrees = 0.0;
  private double armIntegral = 0.0;

  @AutoLogOutput private double lastError = 0.0;

  public Intake() {

    if (hardwareEnabled) {
      armLeader = new TalonFX(Constants.Intake.ArmLeader);
      armFollower = new TalonFX(Constants.Intake.ArmFollower);

      rollerLeft = new SparkFlex(Constants.Intake.RollerLeft, SparkFlex.MotorType.kBrushless);

      rollerRight = new SparkFlex(Constants.Intake.RollerRight, SparkFlex.MotorType.kBrushless);
      armEncoder = new CANcoder(Constants.Intake.ArmEncoder);
      configureArmMotors();
      configureRollers();
      lastArmLimit = armCurrentLimit.get();
      armTargetDegrees = getArmDegrees(); // don't move on start
    }
  }

  /* ===================== Configuration ===================== */

  private void configureArmMotors() {

    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    armFollower.getConfigurator().apply(baseConfig);

    baseConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    baseConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // baseConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // baseConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90.0 / 360.0;

    // baseConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // baseConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

    applyArmCurrentLimit(armCurrentLimit.get());

    armFollower.setControl(new Follower(armLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    // armFollower.setControl(new NeutralOut());
    armLeader.getSupplyCurrent().setUpdateFrequency(100);
    armLeader.optimizeBusUtilization();
  }

  private void configureRollers() {
    rollerLeft.set(0);
    rollerRight.set(0);
  }

  @AutoLogOutput
  private double getArmDegrees() {
    if (!hardwareEnabled) return 0.0;
    return armEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  @AutoLogOutput
  private double getArmRotations() {
    if (!hardwareEnabled) return 0.0;
    return armEncoder.getAbsolutePosition().getValueAsDouble();
  }

  /* ===================== Periodic ===================== */

  @Override
  public void periodic() {

    if (hardwareEnabled) {

      double newLimit = armCurrentLimit.get();
      if (Math.abs(newLimit - lastArmLimit) > 1e-6) {
        applyArmCurrentLimit(newLimit);
        lastArmLimit = newLimit;
      }

      double angle = getArmDegrees();
      double error = armTargetDegrees - angle;

      double p = kP * error;

      armIntegral += error * 0.02; // 20ms loop
      double i = kI * armIntegral;

      double ff = 0.0;

      // Static friction
      if (Math.abs(error) > 0.5) {
        ff += Math.signum(error) * kS;
      }

      // Gravity compensation
      ff += kG * Math.cos(Math.toRadians(angle));

      double voltage = p + i + ff;
      if (Math.abs(error) < 2) {
        voltage = kG * Math.cos(Math.toRadians(angle));
      }
      // Clamp
      voltage = Math.max(-4, Math.min(4, voltage));

      armVoltage.Output = voltage;
      armLeader.setControl(armVoltage);

      lastError = error;
    }

    logTelemetry();
  }

  private void applyArmCurrentLimit(double limit) {

    CurrentLimitsConfigs limits =
        new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(limit);

    armLeader.getConfigurator().apply(limits);
    armFollower.getConfigurator().apply(limits);
  }

  /* ===================== Public Control ===================== */

  // Arm

  public void deploy() {
    moveToAngle(0);
    intake();
  }

  public void retract() {
    moveToAngle(90);
    stopRollers();
  }

  public void goToArbitrayAngle() {
    moveToAngle(arbitraryAngle.get());
  }

  public void stopArm() {
    armCommanded = 0.0;
    setArmPercent(0.0);
  }

  private void setArmPercent(double percent) {
    if (!hardwareEnabled) return;

    armVoltage.Output = percent * 12.0;
    armLeader.setControl(armVoltage);
  }

  private void setArmPercentControl(double percent) {
    if (!hardwareEnabled) return;

    double angle = getArmDegrees();

    // Stop if trying to go past limits
    if (percent > 0 && angle >= ARM_MAX) percent = 0;
    if (percent < 0 && angle <= ARM_MIN) percent = 0;

    armVoltage.Output = -percent * 12.0;

    armLeader.setControl(armVoltage);
  }

  public void moveToAngle(double degrees) {
    armTargetDegrees = Math.max(ARM_MIN, Math.min(ARM_MAX, degrees));
  }

  // Rollers
  public void intake() {
    rollerCommanded = intakePercent.get();
    setRollerPercent(rollerCommanded);
  }

  public void outtake() {
    rollerCommanded = outtakePercent.get();
    setRollerPercent(rollerCommanded);
  }

  public void hold() {
    rollerCommanded = holdPercent.get();
    setRollerPercent(rollerCommanded);
  }

  public void stopRollers() {
    rollerCommanded = 0.0;
    setRollerPercent(0.0);
  }

  private void setRollerPercent(double percent) {
    if (!hardwareEnabled) return;

    rollerLeft.set(percent);
    rollerRight.set(-percent);
  }

  /* ===================== Logging ===================== */

  private void logTelemetry() {

    double armSupplyCurrent = 0.0;
    double rollerLeftCurrent = 0.0;
    double rollerRightCurrent = 0.0;
    double appliedVoltage = 0;
    double armAngle = 0.0;

    if (hardwareEnabled) {
      armSupplyCurrent = armLeader.getSupplyCurrent().getValueAsDouble();
      // rollerLeftCurrent = rollerLeft.getOutputCurrent();
      // rollerRightCurrent = rollerRight.getOutputCurrent();
      armAngle = getArmDegrees();
      appliedVoltage = armLeader.getMotorVoltage().getValueAsDouble();
    }

    Logger.recordOutput("Intake/ArmPercent", armCommanded);
    Logger.recordOutput("Intake/ArmSupplyCurrent", armSupplyCurrent);
    Logger.recordOutput("Intake/RollerPercent", rollerCommanded);
    Logger.recordOutput("Intake/RollerLeftCurrent", rollerLeftCurrent);
    Logger.recordOutput("Intake/RollerRightCurrent", rollerRightCurrent);
    Logger.recordOutput("Intake/HardwareEnabled", hardwareEnabled);
    Logger.recordOutput("Intake/ArmDegrees", armAngle);
    Logger.recordOutput("Intake/AppliedVoltage", appliedVoltage);
  }
}
