package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class ExpandAtMatchStart extends SequentialCommandGroup {
  public ExpandAtMatchStart(Intake intake, Hood hood, Turret turret) {
    addCommands(
        Commands.runOnce(() -> intake.stopRollers(), intake),
        Commands.runOnce(() -> hood.retract(), hood),
        Commands.runOnce(() -> intake.safeAngle(), intake),
        Commands.waitUntil(() -> intake.isAtAngle()),
        Commands.runOnce(() -> turret.setTurretRelativeAngle(0), turret),
        Commands.waitUntil(() -> turret.isAtSetpoint()),
        // Commands.runOnce(() -> turret.zeroTurret(), turret), // redundance since we're now
        // offsetting at startup
        // Commands.runOnce(() -> turret.setTurretRelativeAngle(0), turret),
        Commands.runOnce(() -> intake.retract(), intake));
  }
}
