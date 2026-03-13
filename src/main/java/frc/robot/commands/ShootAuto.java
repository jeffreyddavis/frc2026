package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShootingCoordinator;

public class ShootAuto extends InstantCommand {

  public ShootAuto(ShootingCoordinator coordinator) {
    super(
        () -> {
          coordinator.setRequestShot(true);
        },
        coordinator);
  }
}
