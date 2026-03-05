package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeOut extends InstantCommand {

  public IntakeOut(Intake intake) {
    super(() -> {
      intake.deploy();
      intake.intake();
    }, intake);
  }
}