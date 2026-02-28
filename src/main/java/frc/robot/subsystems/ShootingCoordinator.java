package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShootingCoordinator extends SubsystemBase {

  public enum ShootingMode {
    MANUAL,
    AUTO_AIM
  }

  public enum ShotType {
    NONE,
    SHOOT,
    PASS,
    BLOCKED
  }

  public static boolean trenchOverrideEnabled = false;

  /* ===================== Tunable Targets ===================== */

  // You can move these to Constants later if desired
  private static final double SHOOT_HOOD_MM = 45.0;
  private static final double PASS_HOOD_MM  = 20.0;

  /* ===================== Subsystems ===================== */

  private final Shooter shooter;
  private final Turret turret;
  private final Loader loader;
  private final Spindexer spindexer;
  private final Hood hood;
  private final RobotHealth robotHealth;

  /* ===================== State ===================== */

  private ShootingMode currentMode = ShootingMode.MANUAL;
  private boolean requestShot = false;
  private ShotType currentShotType = ShotType.NONE;

  private double shotRequestStartTime = 0;
  private boolean timingShot = false;
  private static final double SHOOT_FALLBACK_TIME = 0.6; // seconds

  public ShootingCoordinator(
      Shooter shooter,
      Turret turret,
      Hood hood,
      Loader loader,
      Spindexer spindexer,
      RobotHealth robotHealth) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.loader = loader;
    this.spindexer = spindexer;
    this.robotHealth = robotHealth;
  }

  /* ===================== DRIVER INTERFACE ===================== */

  public void setMode(ShootingMode mode) {
    currentMode = mode;
  }

  public void setRequestShot(boolean request) {
    requestShot = request;
  }

  /* ===================== CORE LOGIC ===================== */

  @Override
  public void periodic() {

    currentShotType = determineShotType();

    // ----------------------------
    // HARD trench protection
    // ----------------------------
    boolean trenchBlocked =
        (robotHealth.inTrenchZones || robotHealth.hoodDangerNearTrench)
        && !trenchOverrideEnabled;

    if (trenchBlocked) {
        hood.setPositionMm(0);
        loader.stop();
        spindexer.stop();
        logState(false, false);
        return;
    }

    // ----------------------------
    // Hood Targeting (AUTO_AIM only)
    // ----------------------------
    if (currentMode == ShootingMode.AUTO_AIM) {

      switch (currentShotType) {

        case SHOOT:
          hood.setPositionMm(SHOOT_HOOD_MM);
          break;

        case PASS:
          hood.setPositionMm(PASS_HOOD_MM);
          break;

        case NONE:
        case BLOCKED:
        default:
          // Do nothing special
          break;
      }
    }

    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (requestShot && !timingShot) {
        shotRequestStartTime = now;
        timingShot = true;
    }

    if (!requestShot) {
        timingShot = false;
    }

    boolean shooterReady = shooter.isAtSetpoint();


    boolean timeoutReady =
        timingShot && (now - shotRequestStartTime > SHOOT_FALLBACK_TIME);

    boolean speedGate = shooterReady || timeoutReady;

    // ----------------------------
    // Readiness Check
    // ----------------------------
    boolean readyToFire =
        speedGate
            && turret.isAtSetpoint()
            && hood.isAtSetpoint()
            && robotHealth.fieldReady;

    // ----------------------------
    // Feed Gate
    // ----------------------------
    boolean allowFeed =
        requestShot
            && readyToFire
            && (currentShotType == ShotType.SHOOT
                || currentShotType == ShotType.PASS);

    if (allowFeed) {
      spindexer.feed();
      loader.feed();
    } else {
      spindexer.stop();
      loader.stop();
    }

    logState(readyToFire, allowFeed);
  }

  /* ===================== DECISION TREE ===================== */

  private ShotType determineShotType() {

    if (robotHealth.inTrenchZones || robotHealth.hoodDangerNearTrench) {
      return ShotType.BLOCKED;
    }

    if (!robotHealth.fieldReady) {
      return ShotType.NONE;
    }

    if (robotHealth.inAllianceZone) {
      return ShotType.SHOOT;
    }

    if (robotHealth.inNeutralZone || robotHealth.inOpponentZone) {
      return ShotType.PASS;
    }

    return ShotType.NONE;
  }

  /* ===================== Logging ===================== */

  private void logState(boolean ready, boolean feeding) {
    Logger.recordOutput("Coordinator/Mode", currentMode.toString());
    Logger.recordOutput("Coordinator/ShotType", currentShotType.toString());
    Logger.recordOutput("Coordinator/RequestShot", requestShot);
    Logger.recordOutput("Coordinator/ReadyToFire", ready);
    Logger.recordOutput("Coordinator/Feeding", feeding);
  }
}