package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hood;

public class RetractHood extends InstantCommand {

  public RetractHood(Hood hood) {
    super(() -> hood.retract(), hood);
  }
}