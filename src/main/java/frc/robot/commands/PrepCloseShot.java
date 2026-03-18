package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

public class PrepCloseShot extends InstantCommand {

  public PrepCloseShot(Hood hood, Shooter shoot) {
    super(
        () -> {
          hood.setPositionMm(35);
          shoot.setTargetRPM(4000);
        });
  }
}
