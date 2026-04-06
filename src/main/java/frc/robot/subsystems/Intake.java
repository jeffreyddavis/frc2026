package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
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

  private static final double ARM_MIN = 0;
  private static final double ARM_MAX = 125;

  private static boolean closedloopControl = true;

  private TalonFX rollerLeft;
  // private SparkFlex rollerRight;

  private RelativeEncoder rollerLeftEncoder;
  // private RelativeEncoder rollerRightEncoder;

  private SparkClosedLoopController rollerLeftPID;
  // private SparkClosedLoopController rollerRightPID;

  /* ===================== Tunables ===================== */
  private boolean isAtAngle = false;

  private final LoggedNetworkNumber armCurrentLimit =
      new LoggedNetworkNumber("Intake/ArmSupplyLimit", 40.0);

  // private final LoggedNetworkNumber intakeRPM =
  //    new LoggedNetworkNumber("Intake/RollerIntakeRPM", 2500);
  private final double intakeRPM = 3100;

  private final LoggedNetworkNumber outtakeRPM =
      new LoggedNetworkNumber("Intake/RollerOuttakeRPM", -1500);

  private final LoggedNetworkNumber holdRPM = new LoggedNetworkNumber("Intake/RollerHoldRPM", 500);

  private final LoggedNetworkNumber arbitraryAngle =
      new LoggedNetworkNumber("Intake/arbitraryAngle", 45);

  private final LoggedNetworkNumber jogPower = new LoggedNetworkNumber("Intake/jogPower", .2);

  private final LoggedNetworkNumber maxVolts = new LoggedNetworkNumber("Intake/maxVolts", 3);

  private final LoggedNetworkNumber jamVelocityThreshold =
      new LoggedNetworkNumber("Intake/JamVelocityRPM", 200);

  private final LoggedNetworkNumber jamCurrentThreshold =
      new LoggedNetworkNumber("Intake/JamCurrent", 35);

  private final LoggedNetworkNumber agitateHighAngle =
      new LoggedNetworkNumber("Intake/AgitateHighAngle", 60);

  private final LoggedNetworkNumber agitateLowAngle =
      new LoggedNetworkNumber("Intake/AgitateLowAngle", 3);

  private final LoggedNetworkNumber agitatePeriod =
      new LoggedNetworkNumber("Intake/AgitatePeriod", 0.40); // seconds per half-cycle

  private final LoggedNetworkNumber agitateMaxTime =
      new LoggedNetworkNumber("Intake/AgitateMaxTime", 3.0); // optional cancel timer

  private boolean clearingJam = false;
  private double jamTimer = 0;
  private boolean agitateContinuous = false;
  private boolean agitateHigh = false;
  private double agitateTimer = 0;
  private double agitateTotalTimer = 0;

  private enum ClearMode {
    NONE,
    JAM,
    AGITATE
  }

  @AutoLogOutput private ClearMode clearMode = ClearMode.NONE;
  private double clearTimer = 0;

  // private final LoggedNetworkNumber kP = new LoggedNetworkNumber("Intake/kP", 20);
  private final double kP = .35;

  // private final LoggedNetworkNumber kI = new LoggedNetworkNumber("Intake/kI", .1);
  private final double kI = 0;

  // private final LoggedNetworkNumber kS = new LoggedNetworkNumber("Intake/kS", 1.7);
  private final double kS = 0;

  // private final LoggedNetworkNumber kG = new LoggedNetworkNumber("Intake/kG", 1);
  private final double kG = .01;

  /* ===================== State ===================== */

  private double armCommanded = 0.0;

  @AutoLogOutput private double rollerCommanded = 0.0;
  private double lastArmLimit = 0.0;

  @AutoLogOutput private double armTargetDegrees = 0.0;
  private double armIntegral = 0.0;

  @AutoLogOutput private double rollerRpm = 0;

  @AutoLogOutput private double lastError = 0.0;

  public Intake() {

    if (hardwareEnabled) {
      armLeader = new TalonFX(Constants.Intake.ArmLeader);
      armFollower = new TalonFX(Constants.Intake.ArmFollower);

      rollerLeft = new TalonFX(Constants.Intake.RollerLeft);

      // rollerRight = new SparkFlex(Constants.Intake.RollerRight, SparkFlex.MotorType.kBrushless);
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

  public void requestJamClear() {
    if (clearMode == ClearMode.NONE) {
      clearMode = ClearMode.JAM;
      clearTimer = 0;
    }
  }

  public void startAgitate() {
    agitateContinuous = true;
    agitateHigh = true;
    agitateTimer = 0;
    agitateTotalTimer = 0;
    clearMode = ClearMode.AGITATE;
  }

  public void startTimedAgitate() {
    startAgitate();
    agitateContinuous = false;
  }

  public void stopAgitate() {
    agitateContinuous = false;
    clearMode = ClearMode.NONE;
  }

  private void configureRollers() {

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    // TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    // PID (Slot0 is default)
    leftConfig.Slot0.kP = 0.42;
    leftConfig.Slot0.kI = 0.0;
    leftConfig.Slot0.kD = 0.0;

    // rightConfig.Slot0.kP = 0.00042;
    // rightConfig.Slot0.kI = 0.0;
    // rightConfig.Slot0.kD = 0.0;

    // Optional but recommended: set neutral mode
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply configs
    rollerLeft.getConfigurator().apply(leftConfig);
    // rollerRight.getConfigurator().apply(rightConfig);
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

  public void jogDown() {
    closedloopControl = false;
    DutyCycleOut duty = new DutyCycleOut(-jogPower.get());
    armLeader.setControl(duty);
  }

  public void jogUp() {
    closedloopControl = false;
    DutyCycleOut duty = new DutyCycleOut(jogPower.get());
    armLeader.setControl(duty);
  }

  /* ===================== Periodic ===================== */

  @Override
  public void periodic() {

    if (hardwareEnabled && closedloopControl) {
      // Velocity (in rotations per second by default)
      double rps = rollerLeft.getVelocity().getValueAsDouble();
      boolean tryingToRun = Math.abs(rollerCommanded) > 1000; // or some % threshold

      // Convert to RPM to match your old code
      double rpm = rps * 60.0;
      rollerRpm = rpm;

      // Supply current (what you usually want)
      double current = rollerLeft.getSupplyCurrent().getValueAsDouble();
      boolean jamDetected = Math.abs(rpm) < jamVelocityThreshold.get();
      // && Math.abs(current) > jamCurrentThreshold.get();

      if (jamDetected && tryingToRun) jamTimer += 0.02;
      else jamTimer = 0;

      if (jamTimer > 0.1 && clearMode == ClearMode.NONE) {
        requestJamClear();
      }

      if (clearMode != ClearMode.NONE) {
        runClearRoutine();
      }

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
      voltage = Math.max(-maxVolts.get(), Math.min(maxVolts.get(), voltage));

      armVoltage.Output = voltage;
      armLeader.setControl(armVoltage);

      lastError = error;
    }

    logTelemetry();
  }

  private void runClearRoutine() {

    clearTimer += 0.02;

    if (clearMode == ClearMode.JAM) {

      if (clearTimer < 0.15) {
        // Reverse hard
        setRollerVelocity(-intakeRPM);
      } else if (clearTimer < 0.20) {
        // Optional: brief stop (helps de-tangle)
        setRollerVelocity(0);
      } else {
        // Resume intake
        intake();
        clearMode = ClearMode.NONE;
      }
    } else if (clearMode == ClearMode.AGITATE) {

      intake();

      agitateTimer += 0.02;
      agitateTotalTimer += 0.02;

      if (agitateHigh) {
        moveToAngle(agitateHighAngle.get());
      } else {
        moveToAngle(agitateLowAngle.get());
      }

      if (agitateTimer > agitatePeriod.get()) {
        agitateHigh = !agitateHigh;
        agitateTimer = 0;
      }

      // Optional timeout for manual button
      if (!agitateContinuous && agitateTotalTimer > agitateMaxTime.get()) {
        clearMode = ClearMode.NONE;
      }
    }
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
    moveToAngle(Constants.Intake.deployAngle);
    intake();
  }

  public void barge() { // this is called a barge because Jeffery couldn't remember "depot".
    moveToAngle(Constants.Intake.deployBarge);
    intake();
  }

  public void retract() {
    moveToAngle(Constants.Intake.retractAngle);
    stopRollers();
  }

  public boolean isAtAngle() {
    return isAtAngle;
  }

  public void safeAngle() {
    moveToAngle(Constants.Intake.safeAngle);
  }

  public void readyAngle() {
    moveToAngle(Constants.Intake.readyAngle);
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
    isAtAngle = false;
    armTargetDegrees = Math.max(ARM_MIN, Math.min(ARM_MAX, degrees));
  }

  // Rollers
  public void intake() {
    setRollerVelocity(intakeRPM);
  }

  public void outtake() {
    setRollerVelocity(outtakeRPM.get());
  }

  public void hold() {
    setRollerVelocity(holdRPM.get());
  }

  public void stopRollers() {
    setRollerVelocity(0);
  }

  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0);

  private void setRollerVelocity(double rpm) {

    if (!hardwareEnabled) return;

    rollerCommanded = rpm;

    double rps = rpm / 60.0;

    rollerLeft.setControl(rollerVelocityRequest.withVelocity(rps));
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
      rollerLeftCurrent = rollerLeft.getSupplyCurrent().getValueAsDouble();
      // rollerLeftCurrent = rollerLeft.getOutputCurrent();
      // rollerRightCurrent = rollerRight.getOutputCurrent();
      armAngle = getArmDegrees();
      appliedVoltage = armLeader.getMotorVoltage().getValueAsDouble();

      isAtAngle = (Math.abs(armAngle - armTargetDegrees) < Constants.Intake.armTolerance);
    }

    Logger.recordOutput("Intake/ArmPercent", armCommanded);
    Logger.recordOutput("Intake/ArmSupplyCurrent", armSupplyCurrent);
    Logger.recordOutput("Intake/RollerPercent", rollerCommanded);
    Logger.recordOutput("Intake/RollerLeftCurrent", rollerLeftCurrent);
    // Logger.recordOutput("Intake/RollerRightCurrent", rollerRightCurrent);
    Logger.recordOutput("Intake/HardwareEnabled", hardwareEnabled);
    Logger.recordOutput("Intake/ArmDegrees", armAngle);
    Logger.recordOutput("Intake/AppliedVoltage", appliedVoltage);
  }
}
