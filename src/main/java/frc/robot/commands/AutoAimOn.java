package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShootingCoordinator;
import frc.robot.subsystems.ShootingCoordinator.ShootingMode;
import frc.robot.subsystems.Spindexer;

public class AutoAimOn extends InstantCommand {

  public AutoAimOn(ShootingCoordinator coordinator, Spindexer spindexer) {
    super(
        () -> {
          coordinator.setMode(ShootingMode.AUTO_AIM);
          spindexer.setFeedPercent(5);
        });
  }
}
