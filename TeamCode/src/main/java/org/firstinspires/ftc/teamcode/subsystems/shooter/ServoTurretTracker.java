package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class ServoTurretTracker extends SubsystemBase {

    private final Servo turret;
    private boolean enabled = false;

    private double targetX = 0.0;
    private double targetY = 0.0;

    // ===================== Servo limits =====================
    public static double SERVO_MIN = 0.2;
    public static double SERVO_MAX = 0.8;

    // Robot-relative trim
    public static double TURRET_TRIM_DEG = 0.0;

    // Robot-relative aiming window (your physical turret window)
    public static double TURRET_WINDOW_MIN_DEG = 0.0;
    public static double TURRET_WINDOW_MAX_DEG = 190.0;

    // ===================== ODOMETRY FRAME FIX UPS =====================
    // Use these to match corrected Pinpoint (swap/invert axes) without rewriting math.
    public static boolean SWAP_XY = false;
    public static boolean INVERT_X = false;
    public static boolean INVERT_Y = false;

    // If your heading reference changed (e.g., you previously “-180” everywhere), fix here once.
    public static double HEADING_OFFSET_DEG = 0.0;

    // ===================== Calibration (deg -> servo pos) =====================
    // Your provided calibration (90deg is start/mid)
    public static double A0_DEG = 0.0;   public static double P0 = 0.76;
    public static double A1_DEG = 45.0;  public static double P1 = 0.62;
    public static double A2_DEG = 90.0;  public static double P2 = 0.50;
    public static double A3_DEG = 135.0; public static double P3 = 0.38;
    public static double A4_DEG = 190.0; public static double P4 = 0.20;

    // ===================== Telemetry =====================
    private double lastTargetFieldDeg = 0.0;
    private double lastRobotFieldHeadingDeg = 0.0;
    private double lastTurretRobotDegCmd = 90.0;
    private double lastServoPosCmd = 0.50;

    public ServoTurretTracker(HardwareMap hw, String servoName) {
        turret = hw.get(Servo.class, servoName);
        setPosition(P2); // start at mid
    }

    public void setEnabled(boolean en) { enabled = en; }
    public boolean isEnabled() { return enabled; }

    public void setTargetFieldPointInches(double x, double y) {
        targetX = x;
        targetY = y;
    }

    /** Main update: aim turret at target using current robot field pose. */
    public void update(Pose2D robotPose) {
        if (!enabled) {
            setPosition(P2);
            return;
        }

        // --- Read odom pose (inches) ---
        double rx = robotPose.getX(DistanceUnit.INCH);
        double ry = robotPose.getY(DistanceUnit.INCH);

        // --- Apply axis fixes (dashboard) ---
        if (SWAP_XY) {
            double tmp = rx; rx = ry; ry = tmp;
        }
        if (INVERT_X) rx = -rx;
        if (INVERT_Y) ry = -ry;

        // --- Heading (degrees), normalized 0..360 ---
        double robotFieldHeadingDeg =
                wrap0to360(Math.toDegrees(robotPose.getHeading(AngleUnit.RADIANS)) + HEADING_OFFSET_DEG);

        // --- Field bearing robot -> target (0..360) ---
        double targetFieldDeg = wrap0to360(Math.toDegrees(Math.atan2(targetY - ry, targetX - rx)));

        // --- Robot-relative turret angle we want ---
        // Wrap to (-180..180) first so "closest" selection behaves well.
        double turretRobotDeg = wrapTo180(targetFieldDeg - robotFieldHeadingDeg - TURRET_TRIM_DEG);

        // Your turret window is 0..190, but turretRobotDeg might be negative.
        // Choose representation (deg, deg+360, deg-360) closest to the window, then clamp.
        turretRobotDeg = chooseClosestToWindow(turretRobotDeg, TURRET_WINDOW_MIN_DEG, TURRET_WINDOW_MAX_DEG);
        double chosenTurretRobotDeg = clamp(turretRobotDeg, TURRET_WINDOW_MIN_DEG, TURRET_WINDOW_MAX_DEG);

        // --- Convert angle -> servo pos ---
        double pos = angleDegToServoPos(chosenTurretRobotDeg);
        pos = clamp(pos, SERVO_MIN, SERVO_MAX);
        setPosition(pos);

        // Telemetry latches
        lastTargetFieldDeg = targetFieldDeg;
        lastRobotFieldHeadingDeg = robotFieldHeadingDeg;
        lastTurretRobotDegCmd = chosenTurretRobotDeg;
        lastServoPosCmd = pos;
    }

    private static double chooseClosestToWindow(double deg, double winMin, double winMax) {
        double best = deg;
        double bestCost = costToWindow(deg, winMin, winMax);

        double d1 = deg + 360.0;
        double c1 = costToWindow(d1, winMin, winMax);
        if (c1 < bestCost) { bestCost = c1; best = d1; }

        double d2 = deg - 360.0;
        double c2 = costToWindow(d2, winMin, winMax);
        if (c2 < bestCost) { bestCost = c2; best = d2; }

        return best;
    }

    private static double costToWindow(double x, double lo, double hi) {
        if (x < lo) return lo - x;
        if (x > hi) return x - hi;
        return 0.0;
    }

    /** Piecewise linear angle->pos using 5 points. */
    private double angleDegToServoPos(double angDeg) {
        if (angDeg <= A0_DEG) return P0;
        if (angDeg >= A4_DEG) return P4;

        if (angDeg <= A1_DEG) return lerp(A0_DEG, P0, A1_DEG, P1, angDeg);
        if (angDeg <= A2_DEG) return lerp(A1_DEG, P1, A2_DEG, P2, angDeg);
        if (angDeg <= A3_DEG) return lerp(A2_DEG, P2, A3_DEG, P3, angDeg);
        return lerp(A3_DEG, P3, A4_DEG, P4, angDeg);
    }

    private double lerp(double x0, double y0, double x1, double y1, double x) {
        double t = (x - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    public void setPosition(double pos) {
        turret.setPosition(clamp(pos, SERVO_MIN, SERVO_MAX));
    }

    public double getServoPos() { return lastServoPosCmd; }
    public double getTargetFieldDeg() { return lastTargetFieldDeg; }
    public double getRobotFieldHeadingDeg() { return lastRobotFieldHeadingDeg; }
    public double getTurretRobotDegCmd() { return lastTurretRobotDegCmd; }

    private static double wrap0to360(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    private static double wrapTo180(double deg) {
        deg = (deg + 180.0) % 360.0;
        if (deg < 0) deg += 360.0;
        return deg - 180.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}