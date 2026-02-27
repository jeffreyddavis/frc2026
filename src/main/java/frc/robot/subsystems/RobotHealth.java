package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FlipUtil;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import gg.questnav.questnav.QuestNav;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotHealth extends SubsystemBase {
  @AutoLogOutput public boolean PoseHealthy = true;
  @AutoLogOutput public boolean PoseJump = false;
  @AutoLogOutput public boolean YawDisagreement = false;
  @AutoLogOutput public boolean timeout = false;
  @AutoLogOutput public boolean fieldReady = false;
  @AutoLogOutput public boolean fieldOptimal = false;

  @AutoLogOutput public boolean inAllianceZone = true; // start on your side of the field.
  @AutoLogOutput public boolean inNeutralZone = false;
  @AutoLogOutput public boolean inOpponentZone = false;
  @AutoLogOutput public boolean inTrenchZones = false;

  @AutoLogOutput public boolean hoodDangerNearTrench = false;

  @AutoLogOutput public boolean inActiveShift = false;

  private Drive m_Drivetrain;
  private QuestNavSub m_QuestNav;
  private Vision m_Vision;

  @AutoLogOutput private Pose2d lastPose2d;
  private Pose2d newPose2d;

  private double unhealthyUntil = 0;

  private int fieldStatus = 0; // 0=red, 1=yellow, 2=green
  private int desiredFieldStatus = 0;
  private double statusStableUntil = 0.0;

  private static final double FIELD_STATUS_DEBOUNCE = 0.4; // seconds

  @AutoLogOutput public String fieldStatusText;

  @Override
  public void periodic() {
    newPose2d = m_Drivetrain.getPose(); // get fused position from all sources
    PoseJump = poseHasJumped();

    YawDisagreement = YawDisagreement();

    timeout = poseTimeout();

    if (PoseJump || YawDisagreement || timeout) {
      unhealthyUntil = Timer.getFPGATimestamp() + 0.25;
    }

    PoseHealthy = Timer.getFPGATimestamp() > unhealthyUntil;

    fieldReady = (PoseHealthy && m_Vision.camerasWithPoseCount >= 2);
    fieldOptimal = (PoseHealthy && m_Vision.camerasWithPoseCount >= 3);

    determineFieldState();
    updateZones();

    inActiveShift = HubTracker.isActive();

    lastPose2d = newPose2d; // prep for next periodic;
  }

  private boolean updateWithHysteresis(
      boolean currentState, boolean enterCondition, boolean exitCondition) {
    if (currentState) {
      return !exitCondition; // stay true unless clearly exited
    } else {
      return enterCondition; // only become true if clearly entered
    }
  }

  public void updateZones() {

    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    Pose2d pose = m_Drivetrain.getPose();
    double poseX = FlipUtil.applyX(pose.getX());

    double myZone = (FieldConstants.LinesVertical.allianceZone);
    double neutralZoneNear = (FieldConstants.LinesVertical.neutralZoneNear);
    double neutralZoneFar = (FieldConstants.LinesVertical.neutralZoneFar);
    double oppZone = (FieldConstants.LinesVertical.oppAllianceZone);

    double H = Constants.ZONE_HYSTERESIS;

    // Alliance Zone
    boolean allianceEnter = poseX < (myZone - H);
    boolean allianceExit = poseX > (myZone + H);

    inAllianceZone = updateWithHysteresis(inAllianceZone, allianceEnter, allianceExit);
    boolean neutralEnter = poseX > (neutralZoneNear + H) && poseX <= (neutralZoneFar - H);

    boolean neutralExit = poseX <= (neutralZoneNear - H) || poseX > (neutralZoneFar + H);

    inNeutralZone = updateWithHysteresis(inNeutralZone, neutralEnter, neutralExit);

    boolean trenchEnter =
        (poseX >= (myZone + H) && poseX <= (neutralZoneNear - H))
            || (poseX < (oppZone - H) && poseX > (neutralZoneFar + H));

    boolean trenchExit =
        !((poseX >= (myZone - H) && poseX <= (neutralZoneNear + H))
            || (poseX < (oppZone + H) && poseX > (neutralZoneFar - H)));

    inTrenchZones = updateWithHysteresis(inTrenchZones, trenchEnter, trenchExit);

    boolean oppEnter = poseX > (oppZone + H);
    boolean oppExit = poseX < (oppZone - H);

    inOpponentZone = updateWithHysteresis(inOpponentZone, oppEnter, oppExit);

    Translation2d trench1 =
        FlipUtil.apply(FieldConstants.LeftTrench.openingTopLeft.toTranslation2d());
    Translation2d trench2 =
        FlipUtil.apply(FieldConstants.LeftTrench.openingTopRight.toTranslation2d());
    Translation2d trench3 =
        FlipUtil.apply(FieldConstants.RightTrench.openingTopLeft.toTranslation2d());
    Translation2d trench4 =
        FlipUtil.apply(FieldConstants.RightTrench.openingTopRight.toTranslation2d());

    double D = Constants.TrenchDangerDistance;
    double HD = 0.10; // 10cm hysteresis on distance

    boolean trenchDangerEnter =
        pose.getTranslation().getDistance(trench1) < (D - HD)
            || pose.getTranslation().getDistance(trench2) < (D - HD)
            || pose.getTranslation().getDistance(trench3) < (D - HD)
            || pose.getTranslation().getDistance(trench4) < (D - HD);

    boolean trenchDangerExit =
        pose.getTranslation().getDistance(trench1) > (D + HD)
            && pose.getTranslation().getDistance(trench2) > (D + HD)
            && pose.getTranslation().getDistance(trench3) > (D + HD)
            && pose.getTranslation().getDistance(trench4) > (D + HD);

    hoodDangerNearTrench =
        updateWithHysteresis(hoodDangerNearTrench, trenchDangerEnter, trenchDangerExit);
  }

  public void determineFieldState() {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    // Determine desired state
    if (!PoseHealthy || m_Vision.camerasWithPoseCount <= 1) {
      desiredFieldStatus = 0; // RED
    } else if (m_Vision.camerasWithPoseCount == 2) {
      desiredFieldStatus = 1; // YELLOW
    } else {
      desiredFieldStatus = 2; // GREEN
    }

    // Debounce logic
    if (desiredFieldStatus != fieldStatus) {
      // Only transition if stable long enough
      if (now > statusStableUntil) {
        fieldStatus = desiredFieldStatus;
        statusStableUntil = now + FIELD_STATUS_DEBOUNCE;
      }
    } else {
      // Extend stability window
      statusStableUntil = now + FIELD_STATUS_DEBOUNCE;
    }

    switch (fieldStatus) {
      case 0:
        fieldStatusText = "NOT READY";
        break;
      case 1:
        fieldStatusText = "READY";
        break;
      case 2:
        fieldStatusText = "OPTIMAL";
        break;
    }
  }

  @AutoLogOutput
  public int FieldStatus() {
    return fieldStatus;
  }

  public RobotHealth(Drive drivetrain, QuestNavSub questNaviman, Vision vision) {
    m_Drivetrain = drivetrain;
    m_QuestNav = questNaviman;
    m_Vision = vision;
    lastPose2d = drivetrain.getPose();
  }

  private boolean poseTimeout() {
    return !m_Drivetrain.gyro.isConnected()
        && !m_QuestNav.questNav.isConnected()
        && !m_QuestNav.questNav.isTracking();
  }

  private boolean YawDisagreement() {
    double yawError =
        Math.abs(
            m_Drivetrain
                .getHeading()
                .minus(m_QuestNaviman.questRobotPose2d.getRotation())
                .getDegrees());

    return yawError > Constants.YawWarningTolerance;
  }

  private boolean poseHasJumped() {
    double deltaX = newPose2d.getX() - lastPose2d.getX();
    double deltaY = newPose2d.getY() - lastPose2d.getY();
    double distanceJump = Math.hypot(deltaX, deltaY);

    double maxPhysicallyPossible = Constants.MAX_SPEED * .02 * 2; // .02 is ROBORIO 50Hz.

    return distanceJump > maxPhysicallyPossible;
  }
}
