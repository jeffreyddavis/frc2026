package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;

public class VisionLockCommand extends Command {

  private final Drive drive;
  private double startTime;

  public VisionLockCommand(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > 0.25;
  }
}