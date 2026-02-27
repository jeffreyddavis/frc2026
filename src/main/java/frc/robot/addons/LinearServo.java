package frc.robot.addons;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

/**
 * Actuonix L16-R (RC Linear Servo) wrapper.
 *
 * <p>Manual: 1.0ms = retract, 2.0ms = extend.
 */
public class LinearServo extends Servo {

  private final double lengthMm;
  private double targetPosMm;

  public LinearServo(int channel, double lengthMm) {
    super(channel);
    this.lengthMm = lengthMm;
    this.targetPosMm = 0.0;

    // Force the output pulse range to match Actuonix spec:
    // max, deadbandMax, center, deadbandMin, min
    // We'll make deadband bands narrow around center.
    setBoundsMicroseconds(
        2000, // full forward (extend)
        1550, // deadband max
        1500, // center
        1450, // deadband min
        1000 // full reverse (retract)
        );

    // Start retracted (1.0ms)
    set(0.0);
  }

  /** Command position in mm. 0mm=retract, lengthMm=extend. */
  public void setPositionMm(double positionMm) {
    targetPosMm = MathUtil.clamp(positionMm, 0.0, lengthMm);

    double normalized = targetPosMm / lengthMm; // [0..1]
    set(normalized); // maps to 1.0ms..2.0ms via bounds above
  }

  /** Last commanded position (not measured). */
  public double getTargetPositionMm() {
    return targetPosMm;
  }

  public void retract() {
    setPositionMm(0.0);
  }

  public void extend() {
    setPositionMm(lengthMm);
  }
}
