package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Loader extends SubsystemBase {

  private final boolean hardwareEnabled = Constants.Loader.HardwareEnabled;

  private SparkFlex motor;

  /* ===================== Tunables ===================== */

  private final LoggedNetworkNumber feedPercent = new LoggedNetworkNumber("Loader/FeedPercent", .5);

  private final LoggedNetworkNumber reversePercent =
      new LoggedNetworkNumber("Loader/ReversePercent", -0.5);

  /* ===================== State ===================== */

  private boolean running = false;
  private double commandedPercent = 0.0;

  public Loader() {

    if (hardwareEnabled) {
      motor =
          new SparkFlex(
              Constants.Loader.Motor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    }
  }

  @Override
  public void periodic() {

    logTelemetry();
  }

  /* ===================== Public Control ===================== */

  public void feed() {
    running = true;
    commandedPercent = feedPercent.get();
    setPercentOutput(commandedPercent);
  }

  public void reverse() {
    running = true;
    commandedPercent = reversePercent.get();
    setPercentOutput(commandedPercent);
  }

  public void stop() {
    running = false;
    commandedPercent = 0.0;
    setPercentOutput(0.0);
  }

  public void setManualPercent(double percent) {
    running = true;
    commandedPercent = percent;
    setPercentOutput(percent);
  }

  private void setPercentOutput(double percent) {
    if (!hardwareEnabled) return;

    motor.set(percent);
  }

  /* ===================== Logging ===================== */

  private void logTelemetry() {

    double velocityRPM = 0.0;
    double current = 0.0;

    if (hardwareEnabled) {

      velocityRPM = motor.getEncoder().getVelocity();
      current = motor.getOutputCurrent();
    }

    Logger.recordOutput("Loader/VelocityRPM", velocityRPM);
    Logger.recordOutput("Loader/VelocityRPS", velocityRPM / 60.0);
    Logger.recordOutput("Loader/Current", current);
    Logger.recordOutput("Loader/CommandedPercent", commandedPercent);
    Logger.recordOutput("Loader/Running", running);
    Logger.recordOutput("Loader/HardwareEnabled", hardwareEnabled);
  }
}
