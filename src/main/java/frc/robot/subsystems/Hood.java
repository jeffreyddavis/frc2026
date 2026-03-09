package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.addons.LinearServo;
import java.util.NavigableMap;
import java.util.TreeMap;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hood extends SubsystemBase {

  private final boolean hardwareEnabled = Constants.Hood.HardwareEnabled;

  private static final NavigableMap<Double, Double> ANGLE_TO_MM = new TreeMap<>();

  static {
    // degrees -> actuator mm (MEASURE THESE)
    ANGLE_TO_MM.put(35.0, 18.0);
    ANGLE_TO_MM.put(40.0, 25.0);
    ANGLE_TO_MM.put(45.0, 33.0);
    ANGLE_TO_MM.put(50.0, 42.0);
    ANGLE_TO_MM.put(55.0, 52.0);
    ANGLE_TO_MM.put(60.0, 63.0);
  }

  private LinearServo m_leftHood;
  private LinearServo m_rightHood;

  // Track commanded state (since we don’t have feedback)
  @AutoLogOutput private double lastCommandedMm = 0.0;
  private double motionCompleteTime = 0.0;

  // Spec: 20 mm/sec
  private static final double MM_PER_SECOND = 20.0;

  public Hood() {
    if (hardwareEnabled) {
      m_leftHood = new LinearServo(0, 100);
      m_rightHood = new LinearServo(1, 100);
    }
  }

  /* ===================== Public API ===================== */

  public void setPositionAngle(double degrees) {
    double mm = angleToMm(degrees);
    setPositionMm(mm);
  }

  public void setPositionMm(double positionMm) {

    // Estimate time to move
    double distance = Math.abs(positionMm - lastCommandedMm);
    // if (distance <= Constants.Hood.distanceToleranceMM)
    //  return; // don't command the servo repeatedly
    double travelTime = distance / MM_PER_SECOND;

    lastCommandedMm = positionMm;
    motionCompleteTime = Timer.getFPGATimestamp() + travelTime;

    if (hardwareEnabled) {
      m_leftHood.setPositionMm(positionMm);
      m_rightHood.setPositionMm(positionMm);
    }
  }

  public void testServoForward() {
    if (!hardwareEnabled) return;

    m_leftHood.extend();
    m_rightHood.extend();
  }

  public void testServoMiddle() {
    setPositionMm(50);
  }

  public void testServoBackward() {
    if (!hardwareEnabled) return;

    m_leftHood.retract();
    m_rightHood.retract();
  }

  public void retract() {
    m_leftHood.retract();
    m_rightHood.retract();
  }

  public void extendFully() {
    m_leftHood.extend();
    m_rightHood.extend();
  }

  public void incrementUp() {
    lastCommandedMm++;
    setPositionMm(lastCommandedMm);
  }

  public void incrementDown() {
    lastCommandedMm--;
    setPositionMm(lastCommandedMm);
  }

  public boolean isAtSetpoint() {
    return Timer.getFPGATimestamp() >= motionCompleteTime;
  }

  /* ===================== Helpers ===================== */

  private double angleToMm(double degrees) {

    degrees = MathUtil.clamp(degrees, ANGLE_TO_MM.firstKey(), ANGLE_TO_MM.lastKey());

    var lower = ANGLE_TO_MM.floorEntry(degrees);
    var upper = ANGLE_TO_MM.ceilingEntry(degrees);

    if (lower == null) return upper.getValue();
    if (upper == null) return lower.getValue();
    if (lower.getKey().equals(upper.getKey())) {
      return lower.getValue();
    }

    double t = (degrees - lower.getKey()) / (upper.getKey() - lower.getKey());

    return MathUtil.interpolate(lower.getValue(), upper.getValue(), t);
  }
}
