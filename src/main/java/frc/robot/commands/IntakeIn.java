package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeIn extends InstantCommand {

  public IntakeIn(Intake intake) {
    super(() -> {
      intake.retract();
      intake.stopRollers();
    }, intake);
  }
}