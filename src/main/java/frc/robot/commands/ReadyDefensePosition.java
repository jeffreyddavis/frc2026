package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShootingCoordinator;
import frc.robot.subsystems.ShootingCoordinator.ShootingMode;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;

public class ReadyDefensePosition extends SequentialCommandGroup {
  public ReadyDefensePosition(
      Intake intake,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Loader loader,
      Shooter shooter,
      ShootingCoordinator coordinator) {
    addCommands(
        // Commands.runOnce(() -> turret.zeroTurret()),
        Commands.runOnce(() -> coordinator.setMode(ShootingMode.MANUAL), coordinator),
        Commands.runOnce(() -> intake.stopRollers(), intake),
        Commands.runOnce(() -> hood.neutralPosition(), hood),
        Commands.runOnce(() -> intake.safeAngle(), intake),
        Commands.waitUntil(() -> intake.isAtAngle()),
        Commands.runOnce(
            () -> turret.setTurretRelativeAngle(Constants.Turret.startingDegrees), turret),
        Commands.waitUntil(() -> turret.isAtSetpoint()),
        Commands.runOnce(() -> intake.readyAngle(), intake),
        Commands.runOnce(() -> spindexer.stop(), spindexer),
        Commands.runOnce(() -> loader.stop(), loader),
        Commands.runOnce(() -> shooter.disable(), loader),
        Commands.runOnce(() -> turret.stop(), turret));
  }
}
