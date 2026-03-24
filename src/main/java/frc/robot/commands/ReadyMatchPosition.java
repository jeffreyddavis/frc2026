package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class ReadyMatchPosition extends SequentialCommandGroup {
  public ReadyMatchPosition(Intake intake, Hood hood, Turret turret) {
    addCommands(
        Commands.runOnce(() -> turret.zeroTurret()),
        Commands.runOnce(() -> intake.stopRollers(), intake),
        Commands.runOnce(() -> hood.retract(), hood),
        Commands.runOnce(() -> intake.safeAngle(), intake),
        Commands.waitUntil(() -> intake.isAtAngle()),
        Commands.runOnce(
            () -> turret.setTurretRelativeAngle(Constants.Turret.startingDegrees), turret),
        Commands.waitUntil(() -> turret.isAtSetpoint()),
        Commands.runOnce(() -> intake.readyAngle(), intake));
  }
}
