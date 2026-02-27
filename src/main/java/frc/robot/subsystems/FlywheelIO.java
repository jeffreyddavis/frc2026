package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double targetRPS = 0.0;
  }

  void updateInputs(FlywheelIOInputs inputs);

  void setOpenLoopVolts(double volts);
  void setVelocityRPS(double rps);
}