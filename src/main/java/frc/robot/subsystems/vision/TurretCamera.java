package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import java.util.*;
import java.util.stream.Collectors;

/**
 * TurretCameraSubsystem
 *
 * <p>Self-contained vision system for: - turret-relative yaw aiming - distance-to-hub calculation
 * using AprilTags
 *
 * <p>DOES NOT depend on global pose estimator.
 */
public class TurretCamera extends SubsystemBase {

  // ===================== CONFIG =====================

  private static final double YAW_TOLERANCE_RAD = Math.toRadians(1.5);
  private static final int REQUIRED_STABLE_FRAMES = 3;
  private static final int DISTANCE_FILTER_SIZE = 5;

  // ===================== DEPENDENCIES =====================

  private final AprilTagFieldLayout layout = AprilTagLayoutType.OFFICIAL.getLayout();

  // You must provide this
  private final TurretSupplier turretSupplier;
  private final VisionSupplier visionSupplier;

  // ===================== STATE =====================

  private boolean hasTarget = false;
  private double yawErrorRad = 0.0;
  private double distanceMeters = 0.0;
  private boolean isStable = false;

  private final Deque<Double> distanceHistory = new ArrayDeque<>();
  private int stableFrameCount = 0;

  // ===================== CONSTRUCTOR =====================

  public TurretCamera(TurretSupplier turretSupplier, VisionSupplier visionSupplier) {

    this.turretSupplier = turretSupplier;
    this.visionSupplier = visionSupplier;
  }

  // ===================== PERIODIC =====================

  @Override
  public void periodic() {
    var targets = visionSupplier.getTargets();

    if (targets.isEmpty()) {
      resetState();
      return;
    }

    List<Double> distances = new ArrayList<>();
    double bestYaw = 0.0;
    boolean yawSet = false;

    for (var target : targets) {
      int tagId = target.getFiducialId();

      Optional<Pose3d> tagPoseOpt = layout.getTagPose(tagId);
      if (tagPoseOpt.isEmpty()) continue;

      Pose3d tagPose = tagPoseOpt.get();
      Transform3d cameraToTag = target.getBestCameraToTarget();

      // ===================== STEP 1: camera pose =====================
      Pose3d cameraPose = tagPose.transformBy(cameraToTag.inverse());

      // ===================== STEP 2: robot pose =====================
      Transform3d robotToCamera = getRobotToCameraTransform(turretSupplier.getTurretAngleRad());

      Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

      // ===================== STEP 3: distance =====================
      Translation2d robotXY = robotPose.getTranslation().toTranslation2d();

      Translation2d hubXY = FieldConstants.Hub.topCenterPoint.toTranslation2d();

      double distance = robotXY.getDistance(hubXY);

      distances.add(distance);

      // ===================== YAW =====================
      // Use best target for yaw (first valid one)
      if (!yawSet) {
        bestYaw = Math.toRadians(target.getYaw());
        yawSet = true;
      }
    }

    if (distances.isEmpty()) {
      resetState();
      return;
    }

    // ===================== DISTANCE FILTER =====================
    double medianDistance = median(distances);

    distanceHistory.addLast(medianDistance);
    if (distanceHistory.size() > DISTANCE_FILTER_SIZE) {
      distanceHistory.removeFirst();
    }

    distanceMeters = average(distanceHistory);

    // ===================== YAW =====================
    yawErrorRad = bestYaw;
    hasTarget = true;

    // ===================== STABILITY =====================
    if (Math.abs(yawErrorRad) < YAW_TOLERANCE_RAD) {
      stableFrameCount++;
    } else {
      stableFrameCount = 0;
    }

    isStable = stableFrameCount >= REQUIRED_STABLE_FRAMES;
  }

  // ===================== TRANSFORMS =====================

  /** Computes robot -> camera transform based on turret rotation. */
  private Transform3d getRobotToCameraTransform(double turretAngleRad) {

    // TODO: Replace these with your real constants
    Transform3d robotToTurret = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d());

    Transform3d turretToCamera =
        new Transform3d(
            new Translation3d(0.2, 0.0, 0.3), // example
            new Rotation3d());

    Rotation3d turretRotation = new Rotation3d(0, 0, turretAngleRad);

    Transform3d rotatedTurretToCamera =
        new Transform3d(
            turretToCamera.getTranslation(), turretRotation.plus(turretToCamera.getRotation()));

    return robotToTurret.plus(rotatedTurretToCamera);
  }

  // ===================== HELPERS =====================

  private void resetState() {
    hasTarget = false;
    isStable = false;
    stableFrameCount = 0;
    distanceHistory.clear();
  }

  private double median(List<Double> values) {
    List<Double> sorted = values.stream().sorted().collect(Collectors.toList());
    int mid = sorted.size() / 2;
    return sorted.get(mid);
  }

  private double average(Collection<Double> values) {
    return values.stream().mapToDouble(d -> d).average().orElse(0.0);
  }

  // ===================== PUBLIC API =====================

  public boolean hasTarget() {
    return hasTarget;
  }

  public double getYawErrorRad() {
    return yawErrorRad;
  }

  public double getDistanceMeters() {
    return distanceMeters;
  }

  public boolean isStable() {
    return isStable;
  }

  // ===================== INTERFACES =====================

  /** Abstract turret dependency */
  public interface TurretSupplier {
    double getTurretAngleRad();
  }

  /** Abstract vision dependency */
  public interface VisionSupplier {
    List<VisionTarget> getTargets();
  }

  /** Minimal target interface (adapt to Photon/Limelight/etc.) */
  public interface VisionTarget {
    int getFiducialId();

    Transform3d getBestCameraToTarget();

    double getYaw(); // degrees
  }
}
