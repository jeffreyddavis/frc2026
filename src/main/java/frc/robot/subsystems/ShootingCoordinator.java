package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FlipUtil;
import frc.robot.shot.ShotController;
import frc.robot.shot.ShotSolution;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingCoordinator extends SubsystemBase {

  public enum ShootingMode {
    MANUAL,
    AUTO_AIM,
    DISTANCE_ONLY
  }

  public enum ShotType {
    NONE,
    SHOOT,
    PASS,
    BLOCKED
  }

  public static boolean trenchOverrideEnabled = false;

  @AutoLogOutput private Translation2d bottomCorner;
  @AutoLogOutput private Translation2d topCorner;

  @AutoLogOutput Translation2d chosen;

  /* ===================== Subsystems ===================== */

  private final Shooter shooter;
  private final Turret turret;
  private final Loader loader;
  private final Spindexer spindexer;
  private final Hood hood;
  private final RobotHealth robotHealth;
  private final ShotController shotController;
  private final Drive drive;

  /* ===================== State ===================== */

  @AutoLogOutput private ShootingMode currentMode = ShootingMode.MANUAL;
  private boolean requestShot = false;
  @AutoLogOutput private ShotType currentShotType = ShotType.NONE;

  // private boolean timingShot = false;

  @AutoLogOutput private Translation2d target;
  @AutoLogOutput private Translation2d Unflippedtarget;

  @AutoLogOutput private boolean trenchBlocked = false;

  @AutoLogOutput private double hoodTrim = 0;
  @AutoLogOutput private double shotTrim = 0.0;

  public ShootingCoordinator(
      Shooter shooter,
      Turret turret,
      Hood hood,
      Loader loader,
      Spindexer spindexer,
      RobotHealth robotHealth,
      ShotController shotController,
      Drive drive) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.loader = loader;
    this.spindexer = spindexer;
    this.robotHealth = robotHealth;
    this.shotController = shotController;
    this.drive = drive;
  }

  /* ===================== DRIVER INTERFACE ===================== */

  public void setMode(ShootingMode mode) {
    currentMode = mode;
  }

  public void setRequestShot(boolean request) {
    requestShot = request;
  }

  public void incrementUp() {
    hoodTrim += 0.5;
  }

  public void incrementDown() {
    hoodTrim -= 0.5;
  }

  public void trimLeft() {
    shotTrim += 0.5;
  }

  public void trimRight() {
    shotTrim -= 0.5;
  }

  /* ===================== CORE LOGIC ===================== */

  private Translation2d getPassTarget() {

    // Define in BLUE coordinate space
    bottomCorner = new Translation2d(3.0, 3.0);

    topCorner = new Translation2d(3.0, FieldConstants.fieldWidth - 3.0);

    // Decide which corner is closer to robot
    double robotY = drive.getPose().getY();

    chosen =
        Math.abs(robotY - bottomCorner.getY()) < Math.abs(robotY - topCorner.getY())
            ? bottomCorner
            : topCorner;

    // Flip automatically for red alliance
    // return chosen;
    chosen = new Translation2d(FlipUtil.applyX(chosen.getX()), chosen.getY());
    return chosen;
    // return FlipUtil.applyX(chosen);
  }

  @Override
  public void periodic() {

    currentShotType = determineShotType();

    // ----------------------------
    // HARD trench protection
    // ----------------------------
    trenchBlocked = robotHealth.hoodDangerNearTrench && !trenchOverrideEnabled;

    if (trenchBlocked && !DriverStation.isAutonomous()) // allow autoshots.
    hood.neutralPosition();
    /*
       if (trenchBlocked) {
         hood.setPositionMm(0);
         loader.stop();
         spindexer.stop();
         logState(false, false);
         return;
       }
    */

    switch (currentShotType) {
      case SHOOT:
        Unflippedtarget = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
        target = FlipUtil.apply(Unflippedtarget);
        // target = Unflippedtarget; // ????
        break;
      case PASS:
        target = getPassTarget();
        break;

      case NONE:
      case BLOCKED:
      default:
        // keep hood safe if we don't know where we are
        hood.neutralPosition();
        break;
    }

    // ----------------------------
    // Hood Targeting (AUTO_AIM only)
    // ----------------------------
    if (currentMode == ShootingMode.AUTO_AIM || currentMode == ShootingMode.DISTANCE_ONLY) {

      switch (currentShotType) {
        case SHOOT:
          ShotSolution solution =
              shotController.calculate(
                  getTurretFieldPosition(), drive.getFieldRelativeVelocity(), target);

          if (currentMode == ShootingMode.AUTO_AIM) {
            turret.setFieldTargetAngle(
                solution.turretDegrees() + shotTrim,
                drive.getRotation(),
                Math.toDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond));
          }
          hood.setPositionMm(solution.hoodDegrees() + hoodTrim);
          // if (!DriverStation.isAutonomous())
          shooter.setTargetRPM(solution.shooterRPM());
          // else shooter.setTargetRPM(500);

          break;

        case PASS:
          ShotSolution passSolution =
              shotController.calculate(
                  getTurretFieldPosition(), drive.getFieldRelativeVelocity(), target);
          if (currentMode == ShootingMode.AUTO_AIM) {
            turret.setFieldTargetAngle(
                passSolution.turretDegrees() + shotTrim,
                drive.getRotation(),
                Math.toDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond));
          }
          hood.setPositionMm(passSolution.hoodDegrees() + hoodTrim);
          shooter.setTargetRPM(passSolution.shooterRPM());
          break;

        case NONE:
        case BLOCKED:
        default:
          // Do nothing special
          hood.neutralPosition();
          break;
      }
    }

    // double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    /*
    if (requestShot && !timingShot) {
      shotRequestStartTime = now;
      timingShot = true;
    }

    if (!requestShot) {
      timingShot = false;
    } */

    // boolean shooterReady = shooter.isAtSetpoint();
    // boolean shooterReady = true;

    // boolean timeoutReady = timingShot && (now - shotRequestStartTime > SHOOT_FALLBACK_TIME);

    // boolean speedGate = shooterReady || timeoutReady;

    // ----------------------------
    // Readiness Check
    // ----------------------------
    boolean readyToFire = true;
    // speedGate && turret.isAtSetpoint() && hood.isAtSetpoint() && robotHealth.fieldReady;

    // ----------------------------
    // Feed Gate
    // ----------------------------
    boolean allowFeed =
        requestShot
            && readyToFire
            && (currentShotType == ShotType.SHOOT || currentShotType == ShotType.PASS);

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

    // boolean trenchBlocked =
    //    (robotHealth.inTrenchZones || robotHealth.hoodDangerNearTrench) && !trenchOverrideEnabled;

    // if (trenchBlocked) {
    //  return ShotType.BLOCKED;
    // }

    // if (!robotHealth.fieldReady) {
    //  return ShotType.NONE;
    // }

    if (robotHealth.inAllianceZone) {
      return ShotType.SHOOT;
    }

    if (robotHealth.inNeutralZone || robotHealth.inOpponentZone) {
      return ShotType.PASS;
    }

    return ShotType.NONE;
  }

  public void resetTrim() {
    // rpmTrimPercent.set(0.0);
    hoodTrim = 0;
  }

  private Translation2d getTurretFieldPosition() {

    Translation2d robotTranslation = drive.getPose().getTranslation();

    Translation2d turretOffsetField = // new Translation2d().rotateBy(drive.getRotation());
        Constants.Turret.turretOffset.div(1.5).toTranslation2d().rotateBy(drive.getRotation());

    return robotTranslation.plus(turretOffsetField);
  }

  /* ===================== Logging ===================== */

  private void logState(boolean ready, boolean feeding) {
    Logger.recordOutput("Coordinator/Mode", currentMode.toString());
    Logger.recordOutput("Coordinator/ShotType", currentShotType.toString());
    Logger.recordOutput("Coordinator/RequestShot", requestShot);
    Logger.recordOutput("Coordinator/ReadyToFire", ready);
    Logger.recordOutput("Coordinator/Feeding", feeding);
    Logger.recordOutput("Coordinator/TurretFieldPosition", getTurretFieldPosition());
  }
}
