package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Spindexer;

public class StopShoot extends InstantCommand {

  public StopShoot(Loader loader, Spindexer spindexer) {
    super(
        () -> {
          loader.stop();
          spindexer.stop();
        },
        loader,
        spindexer);
  }
}
