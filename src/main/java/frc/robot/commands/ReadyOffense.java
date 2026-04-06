package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShootingCoordinator;
import frc.robot.subsystems.ShootingCoordinator.ShootingMode;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;

public class ReadyOffense extends SequentialCommandGroup {
  public ReadyOffense(
      Intake intake,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Loader loader,
      Shooter shooter,
      ShootingCoordinator coordinator) {
    addCommands(
        // Commands.runOnce(() -> turret.zeroTurret()),

        Commands.runOnce(() -> intake.safeAngle(), intake),
        Commands.runOnce(() -> shooter.enableClosedLoop(), loader),
        Commands.runOnce(() -> coordinator.setMode(ShootingMode.AUTO_AIM), coordinator),
        Commands.runOnce(() -> spindexer.setFeedPercent(.25), spindexer));
  }
}
