package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeBarge extends InstantCommand {

  public IntakeBarge(Intake intake) {
    super(
        () -> {
          intake.barge();
          intake.intake();
        },
        intake);
  }
}
