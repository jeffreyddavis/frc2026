package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShootingCoordinator;
import frc.robot.subsystems.ShootingCoordinator.ShootingMode;

public class AutoAimOn extends InstantCommand {

  public AutoAimOn(ShootingCoordinator coordinator) {
    super(() -> coordinator.setMode(ShootingMode.AUTO_AIM));
  }
}
