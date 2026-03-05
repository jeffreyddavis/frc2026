package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Spindexer;

public class Shoot extends InstantCommand {

  public Shoot(Loader loader, Spindexer spindexer) {
    super(() -> {
      loader.feed();
      spindexer.feed();
    }, loader, spindexer);
  }
}