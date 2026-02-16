package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDFController;

@Deprecated
public class AxonAnalogClosedLoop extends SubsystemBase {
    private final CRServo servo;
    private final AnalogInput absEncoder;

    // Tuning
    private final PIDFController pid = new PIDFController(
            0.0, // kP
            0.0, // kI
            0.0, // kD
            0.0  // kF (often 0 for position, unless you have a model-based FF)
    );

    // Encoder conversion
    private static final double VREF = 3.3;          // REV analog reference is typically 3.3V
    private static final double TWO_PI = Math.PI * 2;

    // Unwrap state
    private double lastAngleRad = 0.0; // 0..2pi
    private int turns = 0;
    private boolean initialized = false;

    // Calibration
    private double offsetRad = 0.0;    // set so your "zero" is correct
    private double gearRatio = 1.0;    // servoTurnsPerMechanismTurn (e.g., 3.0 means 3 servo rev per 1 turret rev)

    // Target (in mechanism radians, not servo radians)
    private double targetMechRad = 0.0;

    public AxonAnalogClosedLoop(HardwareMap hw, String crServoName, String analogName) {
        servo = hw.get(CRServo.class, crServoName);
        absEncoder = hw.get(AnalogInput.class, analogName);
    }

    public void setPID(double p, double i, double d) {
        pid.setPIDF(p, i, d, 0.0);
    }

    public void setOffsetRad(double offsetRad) {
        this.offsetRad = offsetRad;
    }

    public void setGearRatio(double servoTurnsPerMechTurn) {
        this.gearRatio = servoTurnsPerMechTurn;
    }

    /** Mechanism target in radians (continuous). */
    public void setTargetMechRad(double targetMechRad) {
        this.targetMechRad = targetMechRad;
    }

    /** Read wrapped angle (0..2pi) from voltage. */
    private double readWrappedAngleRad() {
        double v = absEncoder.getVoltage();
        double angle = (v / VREF) * TWO_PI;         // 0..2pi (approximately)
        angle = angle - offsetRad;
        // Normalize to 0..2pi for unwrap logic
        angle %= TWO_PI;
        if (angle < 0) angle += TWO_PI;
        return angle;
    }

    /** Continuous mechanism position in radians (unwrapped). */
    public double getMechPosRad() {
        double a = readWrappedAngleRad(); // 0..2pi

        if (!initialized) {
            lastAngleRad = a;
            turns = 0;
            initialized = true;
            return (a / gearRatio);
        }

        double delta = a - lastAngleRad;

        // unwrap detection across 0/2pi boundary
        if (delta > Math.PI) {
            turns -= 1;          // jumped "forward" across wrap in negative direction
        } else if (delta < -Math.PI) {
            turns += 1;          // jumped "backward" across wrap in positive direction
        }

        lastAngleRad = a;

        double continuousServoRad = a + turns * TWO_PI;
        return continuousServoRad / gearRatio;
    }

    /** Call this in your loop. */
    public void update() {
        double pos = getMechPosRad();
        double out = pid.calculate(pos, targetMechRad);

        // Optional: static friction compensation (simple)
        // double kS = 0.0;
        // if (Math.abs(out) > 1e-4) out += Math.signum(out) * kS;

        out = Range.clip(out, -1.0, 1.0);
        servo.setPower(out);
    }

    public void stop() {
        servo.setPower(0.0);
    }
}
