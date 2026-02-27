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

  private final Shooter shooter;
  private final Turret turret;
  private final Loader loader;
  private final Spindexer spindexer;
  private final Hood hood;
  private final RobotHealth robotHealth;

  private ShootingMode currentMode = ShootingMode.MANUAL;
  private boolean requestShot = false;

  private ShotType currentShotType = ShotType.NONE;

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

    boolean readyToFire =
        shooterAtSpeed() && turretAtSetpoint() && hoodAtSetpoint() && robotHealth.fieldReady;

    boolean allowFeed =
        requestShot
            && readyToFire
            && currentShotType != ShotType.NONE
            && currentShotType != ShotType.BLOCKED;

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

    // Hard block in trench
    if (robotHealth.inTrenchZones) {
      return ShotType.BLOCKED;
    }

    if (!robotHealth.fieldReady) {
      return ShotType.NONE;
    }

    // Alliance zone → full shoot
    if (robotHealth.inAllianceZone) {
      return ShotType.SHOOT;
    }

    // Neutral or opponent zone → pass
    if (robotHealth.inNeutralZone || robotHealth.inOpponentZone) {
      return ShotType.PASS;
    }

    return ShotType.NONE;
  }

  /* ===================== State Helpers ===================== */

  private boolean shooterAtSpeed() {
    return shooter.isAtSetpoint();
  }

  private boolean turretAtSetpoint() {
    return turret.isAtSetpoint();
  }

  private boolean hoodAtSetpoint() {
    return hood.isAtSetpoint();
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
