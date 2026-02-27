package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.*;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.AutoLogOutput;

public class QuestNavSub extends SubsystemBase {

  public enum PositionStatus {
    RECEIVING_FROM_ROBOT,
    SENDING_TO_ROBOT,
    IGNORING
  }

  QuestNav questNav = new QuestNav();
  @AutoLogOutput public PositionStatus currentPositionStatus = PositionStatus.RECEIVING_FROM_ROBOT;
  private Drive m_swerve;
  @AutoLogOutput private Pose2d questPose2d;
  @AutoLogOutput public Pose2d questRobotPose2d;
  private Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 2 degrees rotational
          );

  // First, Declare our geometrical transform from the robot center to the Quest
  private Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          Units.inchesToMeters(10),
          Units.inchesToMeters(1),
          Units.inchesToMeters(6.5),
          new Rotation3d(0, 0, Units.degreesToRadians(180.0)));

  public QuestNavSub(Drive swDrivetrain) {
    m_swerve = swDrivetrain;

    Pose2d fieldPose2d = m_swerve.getPose();

    Pose3d fieldPose3d =
        new Pose3d(
            fieldPose2d.getX(),
            fieldPose2d.getY(),
            0.0, // robot on floor
            new Rotation3d(0.0, 0.0, fieldPose2d.getRotation().getRadians()));
    // Transform by the offset to get the Quest pose
    Pose3d questPose = fieldPose3d.transformBy(ROBOT_TO_QUEST);

    questNav.setPose(questPose);
  }

  @Override
  public void periodic() {
    if (currentPositionStatus == PositionStatus.SENDING_TO_ROBOT) {
      // Get the latest pose data frames from the Quest
      PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

      // Loo pover the pose data frames and send them to the pose estimator
      for (PoseFrame questFrame : questFrames) {
        // Make sure the Quest was tracking the pose for this frame
        if (questFrame.isTracking()) {
          // Get the pose of the Quest
          Pose3d questPose = questFrame.questPose3d();
          questPose2d = questPose.toPose2d();
          // Get timestamp for when the data was sent
          double timestamp = questFrame.dataTimestamp();

          // Transform by the mount pose to get your robot pose
          Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

          questRobotPose2d = robotPose.toPose2d();

          // You can put some sort of filtering here if you would like!

          // Add the measurement to our estimator
          m_swerve.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        }
      }
    } else if (currentPositionStatus == PositionStatus.RECEIVING_FROM_ROBOT) {
      Pose2d fieldPose2d = m_swerve.getPose();
      questRobotPose2d = fieldPose2d;

      Pose3d fieldPose3d =
          new Pose3d(
              fieldPose2d.getX(),
              fieldPose2d.getY(),
              0.0, // robot on floor
              new Rotation3d(0.0, 0.0, fieldPose2d.getRotation().getRadians()));

      // Transform by the offset to get the Quest pose
      Pose3d questPose = fieldPose3d.transformBy(ROBOT_TO_QUEST);
      questPose2d = questPose.toPose2d();

      questNav.setPose(questPose);
    }
  }
}
