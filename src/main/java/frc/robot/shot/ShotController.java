package frc.robot.shot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShotController {

  // =====================
  // Live tuning controls
  // =====================

  private final LoggedNetworkNumber tuningDistance =
      new LoggedNetworkNumber("Shot/TuningDistance", 2.0);

  private final LoggedNetworkNumber hoodOffsetDeg =
      new LoggedNetworkNumber("Shot/HoodOffsetDeg", 0.0);

  private final LoggedNetworkNumber rpmOffset = new LoggedNetworkNumber("Shot/RPMOffset", 0.0);

  private final LoggedNetworkNumber enableTuning =
      new LoggedNetworkNumber("Shot/EnableTuning", 0.0);

  // Latency tuning
  private final LoggedNetworkNumber visionLatencySec =
      new LoggedNetworkNumber("Shot/VisionLatencySec", 0.03);

  private final LoggedNetworkNumber extraLatencySec =
      new LoggedNetworkNumber("Shot/ExtraLatencySec", 0.0);

  private final InterpolatingTreeMap<Double, ShotParams> shotMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), (a, b, t) -> a.interpolate(b, t));

  @AutoLogOutput private double shotDistance = 0.0;

  private double minDistance = Double.POSITIVE_INFINITY;
  private double maxDistance = Double.NEGATIVE_INFINITY;

  public enum ShotMode {
    STATIONARY,
    LEAD
  }

  private ShotMode currentMode = ShotMode.STATIONARY;

  public void setShotMode(ShotMode mode) {
    currentMode = mode;
  }

  public void forceDisableTuning() {
    enableTuning.set(0.0);
  }

  public ShotController() {

    // Use helper method so bounds auto-update

    addShotPoint(0.850, new ShotParams(31, 3634.0, 1.1));
    addShotPoint(1.287, new ShotParams(36, 3634.0, 1.1));
    addShotPoint(1.585, new ShotParams(38, 3634.0, 1.1));
    // addShotPoint(1.654, new ShotParams(17, 3634.0, 1.1));
    // addShotPoint(1.854, new ShotParams(40.467, 3634.0, 1.25));
    // addShotPoint(1.962, new ShotParams(44, 3660.0, 1.2));

    // addShotPoint(2.043, new ShotParams(41, 3634, 1.3));

    addShotPoint(2.150, new ShotParams(43, 3682, 1.3));

    addShotPoint(2.407, new ShotParams(45, 3797, 1.3));

    addShotPoint(2.670, new ShotParams(48.3, 3812, 1.3));
    addShotPoint(2.795, new ShotParams(50, 3820, 1.3));
    //  addShotPoint(3.465, new ShotParams(59.5, 3820, 1.3));
    addShotPoint(3.5, new ShotParams(52.5, 3820, 1.3));

    addShotPoint(4.471, new ShotParams(60, 4000, 1.3));

    // addShotPoint(2.458, new ShotParams(43.2, 3820, 1.3));
    // addShotPoint(3.171, new ShotParams(45.04, 3820, 1.3));
    // addShotPoint(3.842, new ShotParams(52.907, 3820, 1.3));
    // addShotPoint(4.569, new ShotParams(60, 4200, 1.3));
    // addShotPoint(4.773, new ShotParams(60, 4300, 1.3));
    // addShotPoint(5.112, new ShotParams(60, 4320, 1.4));
    addShotPoint(5.463, new ShotParams(60, 4641, 1.4));
    addShotPoint(7, new ShotParams(60, 6500, 1.4));
    addShotPoint(8, new ShotParams(60, 6500, 1.4));
  }

  private void addShotPoint(double distance, ShotParams params) {
    shotMap.put(distance, params);

    minDistance = Math.min(minDistance, distance);
    maxDistance = Math.max(maxDistance, distance);
  }

  private double getTotalLatency() {

    double total = visionLatencySec.get() + extraLatencySec.get();

    // Never allow negative
    if (total < 0.0) {
      total = 0.0;
    }

    return total;
  }

  public ShotSolution calculate(
      Translation2d robotPosition, Translation2d robotVelocity, Translation2d goalPosition) {

    if (currentMode == ShotMode.LEAD) {

      return calculateWithLead(robotPosition, robotVelocity, goalPosition, getTotalLatency());

    } else {

      return calculateStationary(robotPosition, goalPosition);
    }
  }

  /* ============================================================
  STATIONARY SHOT
  ============================================================ */

  public ShotSolution calculateStationary(Translation2d robotPosition, Translation2d goalPosition) {

    Translation2d toGoal = goalPosition.minus(robotPosition);
    double distance = toGoal.getNorm();

    if (distance < 1e-4) {
      return new ShotSolution(0, 0, 0);
    }

    boolean tuningAllowed = enableTuning.get() > 0.5 && !DriverStation.isFMSAttached();
    // If tuning mode is enabled, override distance
    if (tuningAllowed) {
      distance = tuningDistance.get();
    }

    ShotParams baseline = getParams(distance);

    double turretDegrees = toGoal.getAngle().getDegrees();

    double hood = baseline.hoodDegrees();
    double rpm = baseline.shooterRPM();

    // Apply live offsets
    hood += hoodOffsetDeg.get();
    rpm += rpmOffset.get();

    // Logging for AdvantageScope
    Logger.recordOutput("Shot/DistanceUsed", distance);
    Logger.recordOutput("Shot/BaselineHood", baseline.hoodDegrees());
    Logger.recordOutput("Shot/BaselineRPM", baseline.shooterRPM());
    Logger.recordOutput("Shot/FinalHood", hood);
    Logger.recordOutput("Shot/FinalRPM", rpm);

    return new ShotSolution(turretDegrees, hood, rpm);
  }

  /* ============================================================
  LEADING SHOT
  ============================================================ */

  public ShotSolution calculateWithLead(
      Translation2d robotPosition,
      Translation2d robotVelocity,
      Translation2d goalPosition,
      double latencySeconds) {

    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencySeconds));

    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();

    if (distance < 1e-4) {
      return new ShotSolution(0, 0, 0);
    }

    ShotParams baseline = getParams(distance);

    double baselineSpeed = (distance / baseline.timeOfFlightSeconds());

    Translation2d baselineVelocityVector = toGoal.div(distance).times(baselineSpeed);

    Translation2d shotVelocity = baselineVelocityVector.minus(robotVelocity);

    double requiredSpeed = shotVelocity.getNorm();
    Rotation2d turretAngle = shotVelocity.getAngle();

    double speedRatio = requiredSpeed / baselineSpeed;
    double correctedRPM = baseline.shooterRPM() * speedRatio;

    return new ShotSolution(turretAngle.getDegrees(), baseline.hoodDegrees(), correctedRPM);
  }

  /* ============================================================
  INTERNAL
  ============================================================ */

  private ShotParams getParams(double distance) {

    double clamped = MathUtil.clamp(distance, minDistance, maxDistance);

    return shotMap.get(clamped);
  }
}
