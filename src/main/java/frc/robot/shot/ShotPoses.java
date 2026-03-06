package frc.robot.shot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.addons.TunableAutoPose;

public class ShotPoses {

  public static final Pose2d CENTER_SHOT = new Pose2d(5.20, 4.10, Rotation2d.fromDegrees(180));

  public static final Pose2d AMP_SIDE_SHOT = new Pose2d(4.85, 5.75, Rotation2d.fromDegrees(165));

  public static final Pose2d SOURCE_SIDE_SHOT =
      new Pose2d(4.85, 2.45, Rotation2d.fromDegrees(-165));

  public static final TunableAutoPose AUTO_SHOT =
      new TunableAutoPose("Auto/AUTO_SHOT", new Pose2d(4.2, 2.3, Rotation2d.fromDegrees(180)));
}
