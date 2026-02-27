package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.addons.LinearServo;
import java.util.NavigableMap;
import java.util.TreeMap;

public class Hood extends SubsystemBase {

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

  LinearServo m_leftHood = new LinearServo(9, 100);
  LinearServo m_rightHood = new LinearServo(8, 100);

  public Hood() {}

  public void setPositionAngle(double degrees) {
    double mm = angleToMm(degrees);
    setPositionMm(mm);
  }

  private double angleToMm(double degrees) {
    // Clamp to LUT range
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

  public void setPositionMm(double position) {
    m_leftHood.setPositionMm(position);
    m_rightHood.setPositionMm(position);
  }

  public void testServoForward() {
    m_leftHood.extend();
    m_rightHood.extend();
  }

  public void testServoMiddle() {
    m_leftHood.setPositionMm(50);
    m_rightHood.setPositionMm(50);
  }

  public void testServoBackward() {
    m_leftHood.retract();
    m_rightHood.retract();
  }
}
