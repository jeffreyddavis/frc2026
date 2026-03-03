package frc.robot.shot;

public record ShotParams(double hoodDegrees, double shooterRPM, double timeOfFlightSeconds) {

  public ShotParams interpolate(ShotParams other, double t) {
    return new ShotParams(
        lerp(this.hoodDegrees, other.hoodDegrees, t),
        lerp(this.shooterRPM, other.shooterRPM, t),
        lerp(this.timeOfFlightSeconds, other.timeOfFlightSeconds, t));
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}
