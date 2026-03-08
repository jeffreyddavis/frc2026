package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

  private final MotionMagicVoltage armMotion = new MotionMagicVoltage(0);

  private static final double ARM_MIN = 2.0;
  private static final double ARM_MAX = 78.0;

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
      new LoggedNetworkNumber("Intake/arbitraryAngle", 20);

  /* ===================== State ===================== */

  private double armCommanded = 0.0;
  private double rollerCommanded = 0.0;
  private double lastArmLimit = 0.0;

  public Intake() {

    if (hardwareEnabled) {
      armLeader = new TalonFX(Constants.Intake.ArmLeader);
      armFollower = new TalonFX(Constants.Intake.ArmFollower);

      rollerLeft = new SparkFlex(Constants.Intake.RollerLeft, SparkFlex.MotorType.kBrushless);

      rollerRight = new SparkFlex(Constants.Intake.RollerRight, SparkFlex.MotorType.kBrushless);
      armEncoder = new CANcoder(Constants.Intake.ArmEncoder);
      configureArmMotors();
      configureRollers();
      syncArmPositionToEncoder();

      lastArmLimit = armCurrentLimit.get();
    }
  }

  /* ===================== Configuration ===================== */

  private void configureArmMotors() {

    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    armFollower.getConfigurator().apply(baseConfig);

    baseConfig.Slot0.kP = 20;
    baseConfig.Slot0.kI = .1;
    baseConfig.Slot0.kD = 0;
    baseConfig.Slot0.kG = .5;
    baseConfig.Slot0.kS = 1.7;
    baseConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    baseConfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
    baseConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    baseConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // baseConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // baseConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90.0 / 360.0;

    // baseConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // baseConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

    FeedbackConfigs feedback = new FeedbackConfigs();

    feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    feedback.SensorToMechanismRatio = 1.0;

    baseConfig.Feedback = feedback;

    armLeader.getConfigurator().apply(baseConfig);

    applyArmCurrentLimit(armCurrentLimit.get());

    armFollower.setControl(new Follower(armLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    // armFollower.setControl(new NeutralOut());
    armLeader.getSupplyCurrent().setUpdateFrequency(100);
    armLeader.optimizeBusUtilization();
  }

  private void syncArmPositionToEncoder() {

    double rotations = armEncoder.getAbsolutePosition().waitForUpdate(0.1).getValueAsDouble();

    armLeader.setPosition(rotations);
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
    armCommanded = deployPercent.get();
    setArmPercent(armCommanded);
  }

  public void retract() {
    // armCommanded = retractPercent.get();
    // setArmPercent(armCommanded);
    // moveToAngle(20);
  }

  public void goToArbitrayAngle() {
    moveToAngle(arbitraryAngle.get());
  }

  public void deploySoon() {
    moveToAngle(90);
  }

  public void retractSoon() {
    moveToAngle(0);
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

    degrees = Math.max(ARM_MIN, Math.min(ARM_MAX, degrees));

    double rotations = degrees / 360.0;
    armLeader.setControl(armMotion.withPosition(rotations));
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
      rollerLeftCurrent = rollerLeft.getOutputCurrent();
      rollerRightCurrent = rollerRight.getOutputCurrent();
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
