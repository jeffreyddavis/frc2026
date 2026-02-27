package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex; // NEO Vortex controller (adjust if needed)
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {

  /* ===================== Hardware ===================== */

  // Arm motors (Kraken X44)
  private final TalonFX armLeader = new TalonFX(Constants.Intake.ArmLeader);

  private final TalonFX armFollower = new TalonFX(Constants.Intake.ArmFollower);

  private final VoltageOut armVoltage = new VoltageOut(0.0);

  // Roller motors (NEO Vortex)
  private final SparkFlex rollerLeft =
      new SparkFlex(Constants.Intake.RollerLeft, SparkFlex.MotorType.kBrushless);

  private final SparkFlex rollerRight =
      new SparkFlex(Constants.Intake.RollerRight, SparkFlex.MotorType.kBrushless);

  /* ===================== Tunables ===================== */

  // Arm speeds
  private final LoggedNetworkNumber deployPercent =
      new LoggedNetworkNumber("Intake/ArmDeployPercent", 0.5);

  private final LoggedNetworkNumber retractPercent =
      new LoggedNetworkNumber("Intake/ArmRetractPercent", -0.5);

  private final LoggedNetworkNumber armCurrentLimit =
      new LoggedNetworkNumber("Intake/ArmSupplyLimit", 40.0);

  // Roller speeds
  private final LoggedNetworkNumber intakePercent =
      new LoggedNetworkNumber("Intake/RollerIntakePercent", 0.7);

  private final LoggedNetworkNumber outtakePercent =
      new LoggedNetworkNumber("Intake/RollerOuttakePercent", -0.6);

  private final LoggedNetworkNumber holdPercent =
      new LoggedNetworkNumber("Intake/RollerHoldPercent", 0.1);

  /* ===================== State ===================== */

  private double armCommanded = 0.0;
  private double rollerCommanded = 0.0;

  private double lastArmLimit;

  public Intake() {
    configureArmMotors();
    configureRollers();
  }

  /* ===================== Configuration ===================== */

  private void configureArmMotors() {

    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armLeader.getConfigurator().apply(baseConfig);
    armFollower.getConfigurator().apply(baseConfig);

    CurrentLimitsConfigs limits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(armCurrentLimit.get());

    armLeader.getConfigurator().apply(limits);
    armFollower.getConfigurator().apply(limits);

    armFollower.setControl(new Follower(armLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    armLeader.getSupplyCurrent().setUpdateFrequency(100);
    armLeader.optimizeBusUtilization();

    lastArmLimit = armCurrentLimit.get();
  }

  private void configureRollers() {
    rollerLeft.set(0);
    rollerRight.set(0);
  }

  /* ===================== Periodic ===================== */

  @Override
  public void periodic() {

    // Live-update arm current limit if changed
    double newLimit = armCurrentLimit.get();
    if (Math.abs(newLimit - lastArmLimit) > 1e-6) {

      CurrentLimitsConfigs limits =
          new CurrentLimitsConfigs()
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentLimit(newLimit);

      armLeader.getConfigurator().apply(limits);
      armFollower.getConfigurator().apply(limits);

      lastArmLimit = newLimit;
    }

    logTelemetry();
  }

  /* ===================== Public Control ===================== */

  // Arm control
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
    armVoltage.Output = percent * 12.0;
    armLeader.setControl(armVoltage);
  }

  // Roller control
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
    rollerLeft.set(percent);
    rollerRight.set(percent);
  }

  /* ===================== Logging ===================== */

  private void logTelemetry() {

    Logger.recordOutput("Intake/ArmPercent", armCommanded);
    Logger.recordOutput("Intake/ArmSupplyCurrent", armLeader.getSupplyCurrent().getValueAsDouble());

    Logger.recordOutput("Intake/RollerPercent", rollerCommanded);
    Logger.recordOutput("Intake/RollerLeftCurrent", rollerLeft.getOutputCurrent());
    Logger.recordOutput("Intake/RollerRightCurrent", rollerRight.getOutputCurrent());
  }
}
