package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class WaitForHoodReady extends Command {

  private final Hood hood;
  private final double timeout;
  private double startTime;

  public WaitForHoodReady(Hood hood, double timeoutSeconds) {
    this.hood = hood;
    this.timeout = timeoutSeconds;
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return hood.isAtSetpoint() || (Timer.getFPGATimestamp() - startTime) > timeout;
  }
}
