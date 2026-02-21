package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class HoodSubsystem extends SubsystemBase {
    private Servo hood;

    // Manual
    private double manualHoodPos;

    // Auto
    private boolean automatic = false;
    private double autoHoodPos = 0.0;

    // Servo limits for this robot (tune these)
    public static double HOOD_SERVO_MIN = 0.05;
    public static double HOOD_SERVO_MAX = 0.80;

    // Distance->pos table (inches)
    public static double D0 = 24,  P0 = 0.18;
    public static double D1 = 48,  P1 = 0.28;
    public static double D2 = 72,  P2 = 0.36;
    public static double D3 = 96,  P3 = 0.46;
    public static double D4 = 120, P4 = 0.8;

    public HoodSubsystem(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "hood");
        manualHoodPos = HOOD_SERVO_MIN;
        hood.setPosition(manualHoodPos);
    }

    public void setAutomatic(boolean enabled) {
        automatic = enabled;
    }

    public boolean isAutomatic() {
        return automatic;
    }

    /** Call every loop after you update robot pose. */
    public void update() {
        double cmd = automatic ? autoHoodPos : manualHoodPos;
        cmd = Range.clip(cmd, HOOD_SERVO_MIN, HOOD_SERVO_MAX);
        hood.setPosition(cmd);
    }

    @Override
    public void periodic() {
        update();
    }

    /** Manual nudge still works when not in automatic mode. */
    public void aiming(boolean increase, boolean decrease) {
        if (automatic) return; // prevent fighting auto
        if (increase) {
            manualHoodPos = Range.clip(manualHoodPos + 0.01, HOOD_SERVO_MIN, HOOD_SERVO_MAX);
        } else if (decrease) {
            manualHoodPos = Range.clip(manualHoodPos - 0.01, HOOD_SERVO_MIN, HOOD_SERVO_MAX);
        }
    }

    /**
     * Compute auto hood pos from robot (x,y) and goal (x,y).
     * Use the same goal point you use for turret aiming.
     */
    public void updateAutoHoodFromField(double robotXIn, double robotYIn, double goalXIn, double goalYIn) {
        double dx = goalXIn - robotXIn;
        double dy = goalYIn - robotYIn;
        double dist = Math.hypot(dx, dy); // inches

        autoHoodPos = interpDistToPos(dist);
    }

    /** Piecewise linear interpolation across the table. */
    private double interpDistToPos(double dist) {
        // clamp outside range
        if (dist <= D0) return P0;
        if (dist >= D4) return P4;

        // find segment
        if (dist <= D1) return lerp(D0, P0, D1, P1, dist);
        if (dist <= D2) return lerp(D1, P1, D2, P2, dist);
        if (dist <= D3) return lerp(D2, P2, D3, P3, dist);
        return lerp(D3, P3, D4, P4, dist);
    }

    private double lerp(double x0, double y0, double x1, double y1, double x) {
        double t = (x - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    public double getHoodPos() {
        return hood.getPosition();
    }
    public void aimScoring(double hoodPos) {
        this.manualHoodPos = hoodPos;
    }

    public double getAutoHoodPos() {
        return autoHoodPos;
    }
}
