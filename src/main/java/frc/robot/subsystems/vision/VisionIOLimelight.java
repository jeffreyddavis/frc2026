package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.*;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {

  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;

  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private String m_name;

  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    m_name = name;
    var table = NetworkTableInstance.getDefault().getTable(name);

    this.rotationSupplier = rotationSupplier;

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();

    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);

    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  public String getName() {
    return m_name;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    // ===================== CONNECTION =====================

    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // ===================== BASIC TARGET =====================

    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // ===================== ORIENTATION (for MegaTag2) =====================

    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault().flush();

    // ===================== DATA STORAGE =====================

    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // ===================== MEGATAG1 =====================

    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

      int tagCount = (int) rawSample.value[7];
      double avgDistance = rawSample.value[9];

      // Collect tag IDs
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      // ===================== NORMAL POSE PIPELINE =====================

      poseObservations.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
              tagCount,
              avgDistance,
              PoseObservationType.MEGATAG_1));

      // ===================== NEW: TURRET TARGET EXTRACTION =====================

      // Only if we actually see tags
      // if (tagCount > 0) {

      //  double tx = txSubscriber.get();
      //  double ty = tySubscriber.get();

      //  for (int i = 11; i < rawSample.value.length; i += 7) {
      //    int tagId = (int) rawSample.value[i];

      //    turretTargets.add(
      //        new TurretTargetObservation(
      //            tagId, Rotation2d.fromDegrees(tx), Rotation2d.fromDegrees(ty), avgDistance));
      //  }
      // }
    }

    // ===================== OPTIONAL MEGATAG2 =====================

    if (false && DriverStation.isEnabled()) {
      for (var rawSample : megatag2Subscriber.readQueue()) {
        if (rawSample.value.length == 0) continue;

        int tagCount = (int) rawSample.value[7];
        double avgDistance = rawSample.value[9];

        for (int i = 11; i < rawSample.value.length; i += 7) {
          tagIds.add((int) rawSample.value[i]);
        }

        poseObservations.add(
            new PoseObservation(
                rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                parsePose(rawSample.value),
                0.0,
                tagCount,
                avgDistance,
                PoseObservationType.MEGATAG_2));
      }
    }

    // ===================== SAVE OUTPUT =====================

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
