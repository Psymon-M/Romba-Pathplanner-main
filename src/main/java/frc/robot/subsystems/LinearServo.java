package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LinearServo subsystem: provides non-blocking, periodic-stepped movement between
 * normalized bounds (0.0 - 1.0). The actuator in this project uses normalized positions
 * where minLength and maxLength are the allowed range (e.g. 0.125 .. 0.75).
 */
public class LinearServo extends SubsystemBase {
    // Hardware
    public final Servo servo = new Servo(1);

    // Public bounds (normalized 0..1). Keep names for compatibility with existing code.
    public double maxLength = 0.45;
    public double minLength = 0.33;

    // current cached position (normalized)
    public double pos = servo.get();

    // Non-blocking movement state
    private double targetPos = pos; // where we're currently moving to
    // Movement speed expressed in millimeters per second (user-requested).
    public double speedMmPerSecond = 1.0; // default, editable
    // Physical travel (mm) corresponding to normalized 0..1. Change to your actuator's travel.
    public double travelMm = 100.0; // default estimate (editable)
    // Position tolerance in millimeters for considering the actuator "at target".
    public double positionToleranceMm = 2.0; // editable

    // Time tracking for smooth stepping
    private double lastTimestamp = Timer.getFPGATimestamp();

    /**
     * Set a target normalized position (0..1). The subsystem will step toward this
     * target inside periodic() at the configured speed.
     */
    public void setTargetNormalized(double target) {
        targetPos = MathUtil.clamp(target, minLength, maxLength);
    }

    /** Convenience: move to the configured top bound (non-blocking). */
    public void moveToTop() {
        setTargetNormalized(maxLength);
    }

    /** Convenience: move to the configured bottom bound (non-blocking). */
    public void moveToBottom() {
        setTargetNormalized(minLength);
    }

    /** Stop motion (hold current position). */
    public void stopMovement() {
        targetPos = servo.get();
    }

    /** Set the movement speed in millimeters per second (absolute value). */
    public void setSpeedMmPerSecond(double speedMm) {
        speedMmPerSecond = Math.abs(speedMm);
    }

    /**
     * True when the actuator is at (or very near) the target position.
     * @param tolerance normalized tolerance (e.g. 0.005)
     */
    public boolean atTarget(double tolerance) {
        return Math.abs(servo.get() - targetPos) <= Math.abs(tolerance);
    }


    @Override
    public void periodic() {
        // Called every scheduler run; step toward target at configured speed.
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        // protect against weird timestamps
        if (dt <= 0) {
            lastTimestamp = now;
            return;
        }

    double current = servo.get();
    double delta = targetPos - current;
    // convert mm/sec -> normalized/sec
    double speedNormalizedPerSecond = (speedMmPerSecond / Math.max(1e-6, travelMm));
    double maxStep = speedNormalizedPerSecond * dt;

        if (Math.abs(delta) <= maxStep) {
            // close enough: snap to target
            servo.set(targetPos);
            pos = targetPos;
        } else {
            // step toward the target without overshooting
            double next = current + Math.signum(delta) * maxStep;
            next = MathUtil.clamp(next, minLength, maxLength);
            servo.set(next);
            pos = next;
        }

        lastTimestamp = now;
    }
}