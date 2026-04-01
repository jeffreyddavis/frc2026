package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class ReadyCompactPosition extends SequentialCommandGroup {
  public ReadyCompactPosition(Intake intake, Hood hood, Turret turret) {
    addCommands(
        Commands.runOnce(() -> intake.stopRollers(), intake),
        Commands.runOnce(() -> hood.neutralPosition(), hood),
        Commands.runOnce(() -> intake.readyAngle(), intake));
  }
}
