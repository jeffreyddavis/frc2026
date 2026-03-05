package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShootingCoordinator;
import frc.robot.subsystems.ShootingCoordinator.ShootingMode;

public class ManualAiming extends InstantCommand {

  public ManualAiming(ShootingCoordinator coordinator) {
    super(() -> coordinator.setMode(ShootingMode.MANUAL));
  }
}