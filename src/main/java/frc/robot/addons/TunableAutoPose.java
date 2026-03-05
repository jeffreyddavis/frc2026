package frc.robot.addons;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableAutoPose {

  private final LoggedNetworkNumber x;
  private final LoggedNetworkNumber y;
  private final LoggedNetworkNumber theta;

  public TunableAutoPose(String name, Pose2d defaultPose) {

    x = new LoggedNetworkNumber(name + "/x", defaultPose.getX());
    y = new LoggedNetworkNumber(name + "/y", defaultPose.getY());
    theta =
        new LoggedNetworkNumber(
            name + "/theta",
            defaultPose.getRotation().getDegrees());
  }

  public Pose2d get() {
    return new Pose2d(
        x.get(),
        y.get(),
        Rotation2d.fromDegrees(theta.get()));
  }
}