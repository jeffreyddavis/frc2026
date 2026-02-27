package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {

    private final TalonFX motor = new TalonFX(Constants.Turret.Motor);
    private final CANcoder encoder = new CANcoder(Constants.Turret.Encoder);

    private final VoltageOut voltageOut = new VoltageOut(0.0);

    /* ===================== Tunables ===================== */

    private final LoggedNetworkNumber targetAngleDeg =
        new LoggedNetworkNumber("Turret/TargetDeg", 0.0);

    private final LoggedNetworkNumber kP =
        new LoggedNetworkNumber("Turret/kP", 0.02);

    private final LoggedNetworkNumber maxOutput =
        new LoggedNetworkNumber("Turret/MaxOutput", 0.4);

    private final LoggedNetworkNumber toleranceDeg =
        new LoggedNetworkNumber("Turret/ToleranceDeg", 2.0);

    /* ===================== State ===================== */

    private boolean closedLoop = false;

    public Turret() {
        configureMotor();
        configureEncoder();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode =
            com.ctre.phoenix6.signals.NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 25;

        motor.getConfigurator().apply(config);
    }

    private void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset =
            Constants.Turret.EncoderOffset;
        encoder.getConfigurator().apply(config);
    }

    /* ===================== Public Control ===================== */

    public void enableClosedLoop() {
        closedLoop = true;
    }

    public void disable() {
        closedLoop = false;
        setPercentOutput(0);
    }

    public void setManualPercent(double percent) {
        closedLoop = false;
        setPercentOutput(percent);
    }

    /* ===================== Core Logic ===================== */

    @Override
    public void periodic() {

        double current = getTurretAngleDegrees();
        double target = targetAngleDeg.get();

        double delta = computeSafeDelta(current, target);

        Logger.recordOutput("Turret/CurrentDeg", current);
        Logger.recordOutput("Turret/TargetDeg", target);
        Logger.recordOutput("Turret/DeltaDeg", delta);

        if (!closedLoop) {
            return;
        }

        if (Math.abs(delta) < toleranceDeg.get()) {
            setPercentOutput(0);
            Logger.recordOutput("Turret/Output", 0.0);
            return;
        }

        double output = delta * kP.get();

        // Clamp output
        double max = maxOutput.get();
        output = Math.max(-max, Math.min(max, output));

        setPercentOutput(output);

        Logger.recordOutput("Turret/Output", output);
    }

    private void setPercentOutput(double percent) {
        voltageOut.Output = percent * 12.0;
        motor.setControl(voltageOut);
    }

    /* ===================== Encoder ===================== */

    public double getTurretAngleDegrees() {
        return encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    /* ===================== Safety & Math ===================== */

    private double normalizeToSigned(double degrees) {
        double angle = degrees % 360.0;
        if (angle >= 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private double computeSafeDelta(double currentDeg, double targetDeg) {

        double current = normalizeToSigned(currentDeg);
        double target  = normalizeToSigned(targetDeg);

        double delta = target - current;

        // Wrap shortest path
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        // Clamp away from forbidden region
        double maxAllowed = 180 - Constants.Turret.FORBIDDEN_BUFFER_DEG;
        if (Math.abs(target) > maxAllowed) {
            target = Math.signum(target) * maxAllowed;
        }

        return delta;
    }
}