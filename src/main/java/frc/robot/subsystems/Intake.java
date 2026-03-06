package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {

  private final boolean hardwareEnabled = Constants.Intake.HardwareEnabled;

  /* ===================== Hardware ===================== */

  private TalonFX armLeader;
  private TalonFX armFollower;

  private final VoltageOut armVoltage = new VoltageOut(0.0);

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

      configureArmMotors();
      configureRollers();

      lastArmLimit = armCurrentLimit.get();
    }
  }

  /* ===================== Configuration ===================== */

  private void configureArmMotors() {

    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armLeader.getConfigurator().apply(baseConfig);
    armFollower.getConfigurator().apply(baseConfig);

    applyArmCurrentLimit(armCurrentLimit.get());

    armFollower.setControl(new Follower(armLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    armLeader.getSupplyCurrent().setUpdateFrequency(100);
    armLeader.optimizeBusUtilization();
  }

  private void configureRollers() {
    rollerLeft.set(0);
    rollerRight.set(0);
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
    armCommanded = retractPercent.get();
    setArmPercent(armCommanded);
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

    if (hardwareEnabled) {
      armSupplyCurrent = armLeader.getSupplyCurrent().getValueAsDouble();
      rollerLeftCurrent = rollerLeft.getOutputCurrent();
      rollerRightCurrent = rollerRight.getOutputCurrent();
    }

    Logger.recordOutput("Intake/ArmPercent", armCommanded);
    Logger.recordOutput("Intake/ArmSupplyCurrent", armSupplyCurrent);
    Logger.recordOutput("Intake/RollerPercent", rollerCommanded);
    Logger.recordOutput("Intake/RollerLeftCurrent", rollerLeftCurrent);
    Logger.recordOutput("Intake/RollerRightCurrent", rollerRightCurrent);
    Logger.recordOutput("Intake/HardwareEnabled", hardwareEnabled);
  }
}
