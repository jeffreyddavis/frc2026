package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class WaitForShooterReady extends Command {

  private final Shooter shooter;
  private final double timeout;
  private double startTime;

  public WaitForShooterReady(Shooter shooter, double timeoutSeconds) {
    this.shooter = shooter;
    this.timeout = timeoutSeconds;
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return shooter.isAtSetpoint() || (Timer.getFPGATimestamp() - startTime) > timeout;
  }
}
