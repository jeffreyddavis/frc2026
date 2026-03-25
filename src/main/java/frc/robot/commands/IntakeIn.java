package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeIn extends SequentialCommandGroup {

  public IntakeIn(Intake intake) {
    addCommands(
        Commands.runOnce(() -> intake.moveToAngle(Constants.Intake.retractAngle), intake),
        Commands.waitUntil(() -> intake.isAtAngle()),
        Commands.runOnce(() -> intake.stopRollers(), intake));
  }
}
