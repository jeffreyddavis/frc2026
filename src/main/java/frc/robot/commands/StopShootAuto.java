package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShootingCoordinator;

public class StopShootAuto extends InstantCommand {

  public StopShootAuto(ShootingCoordinator coordinator, Hood hood) {
    super(
        () -> {
          coordinator.setRequestShot(false);
          coordinator.setMode(ShootingCoordinator.ShootingMode.MANUAL);
          hood.neutralPosition();
        },
        coordinator);
  }
}
