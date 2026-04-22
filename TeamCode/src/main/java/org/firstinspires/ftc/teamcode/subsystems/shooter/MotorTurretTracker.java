package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class MotorTurretTracker extends SubsystemBase {

    private final DcMotorEx turretMotor;
    private final AnalogInput turretAbs;

    private boolean enabled = false;
    private boolean homed = false;

    private double targetX = 0.0;
    private double targetY = 0.0;

    // ===================== Turret geometry / gearing =====================
    public static double MOTOR_REVS_PER_TURRET_REV = 3.0;
    public static double TICKS_PER_MOTOR_REV = 8192.0; // CHANGE THIS to your Melonbotics CPR the hub reports per rev
    public static double ABS_MAX_VOLTAGE = 3.2;        // Melonbotics analog output scale

    // ===================== Motor control =====================
    public static double kP = 0.012;
    public static double kI = 0.0;
    public static double kD = 0.0002;
    public static double kStatic = 0.03;
    public static double MAX_POWER = 0.6;
    public static double ANGLE_TOLERANCE_DEG = 1.0;

    // ===================== Homing / hard stop =====================
    public static boolean USE_HOMING = true;
    public static double HOMING_POWER = -0.15;              // direction toward hard stop
    public static double HOMING_CURRENT_AMPS = 2.5;         // tune this
    public static long HOMING_STALL_TIME_MS = 150;          // tune this
    public static double HOMING_VEL_TICKS_PER_SEC = 20.0;   // near-zero velocity threshold
    public static double HOME_TURRET_ANGLE_DEG = 0.0;       // turret angle when hard stop is hit
    public static double BACKOFF_DEG_AFTER_HOME = 3.0;      // optional back off from stop

    // ===================== Robot-relative trim / turret window =====================
    public static double TURRET_TRIM_DEG = 0.0;
    public static double TURRET_WINDOW_MIN_DEG = 0.0;
    public static double TURRET_WINDOW_MAX_DEG = 180.0;

    // ===================== ODOMETRY FRAME FIX UPS =====================
    public static boolean SWAP_XY = false;
    public static boolean INVERT_X = false;
    public static boolean INVERT_Y = false;
    public static double HEADING_OFFSET_DEG = 0.0;

    // Robot-frame offset from odom origin to turret pivot (inches)
    public static double TURRET_OFFSET_X_IN = 0.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    // ===================== Absolute encoder calibration =====================
    // Offset applied to motor-shaft absolute angle from analog encoder
    public static double MOTOR_ABS_OFFSET_DEG = 0.0;

    // ===================== Telemetry =====================
    private double lastTargetFieldDeg = 0.0;
    private double lastRobotFieldHeadingDeg = 0.0;
    private double lastTurretRobotDegCmd = 90.0;
    private double lastTurretRobotDegMeasured = 90.0;
    private double lastMotorAbsDeg = 0.0;
    private double lastMotorCurrentAmps = 0.0;
    private double lastPowerCmd = 0.0;

    // Internal tracking
    private int homeMotorTicks = 0;          // motor encoder ticks at home hard-stop reference
    private long overCurrentStartMs = -1;

    // Simple PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastUpdateNs = 0;

    public MotorTurretTracker(HardwareMap hw, String motorName, String analogName) {
        turretMotor = hw.get(DcMotorEx.class, motorName);
        turretAbs = hw.get(AnalogInput.class, analogName);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Establish a startup estimate from the analog absolute reading.
        initializeMotorTicksFromAbsolute();
    }

    public void setEnabled(boolean en) {
        enabled = en;
        if (!en) {
            turretMotor.setPower(0.0);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isHomed() {
        return homed;
    }

    public void setTargetFieldPointInches(double x, double y) {
        targetX = x;
        targetY = y;
    }

    /**
     * Call once during init if you want a hard-stop homing routine.
     * Then keep calling update() each loop until isHomed() becomes true.
     */
    public void startHoming() {
        homed = false;
        overCurrentStartMs = -1;
    }

    /**
     * Main update: aim turret at target using current robot field pose.
     */
    public void update(Pose2D robotPose) {
        long nowNs = System.nanoTime();
        double dt = (lastUpdateNs == 0) ? 0.02 : (nowNs - lastUpdateNs) / 1e9;
        lastUpdateNs = nowNs;

        lastMotorCurrentAmps = turretMotor.getCurrent(CurrentUnit.AMPS);
        lastMotorAbsDeg = getMotorAbsDeg();

        if (!enabled) {
            turretMotor.setPower(0.0);
            lastPowerCmd = 0.0;
            return;
        }

        if (USE_HOMING && !homed) {
            runHoming();
            return;
        }

        // --- Read odom pose (inches) ---
        double rx = robotPose.getX(DistanceUnit.INCH);
        double ry = robotPose.getY(DistanceUnit.INCH);

        // --- Apply axis fixes ---
        if (SWAP_XY) {
            double tmp = rx; rx = ry; ry = tmp;
        }
        if (INVERT_X) rx = -rx;
        if (INVERT_Y) ry = -ry;

        // --- Heading in radians and degrees ---
        double robotHeadingRad = robotPose.getHeading(AngleUnit.RADIANS) + Math.toRadians(HEADING_OFFSET_DEG);
        double robotFieldHeadingDeg = wrap0to360(Math.toDegrees(robotHeadingRad));

        // --- Compute turret pivot field position from robot pose + rotated offset ---
        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);

        double pivotX = rx + (TURRET_OFFSET_X_IN * cosH - TURRET_OFFSET_Y_IN * sinH);
        double pivotY = ry + (TURRET_OFFSET_X_IN * sinH + TURRET_OFFSET_Y_IN * cosH);

        // --- Field bearing turretPivot -> target ---
        double targetFieldDeg = wrap0to360(Math.toDegrees(Math.atan2(targetY - pivotY, targetX - pivotX)));

        // --- Desired robot-relative turret angle ---
        double turretRobotDeg = wrapTo180(targetFieldDeg - robotFieldHeadingDeg - TURRET_TRIM_DEG);

        turretRobotDeg = chooseClosestToWindow(turretRobotDeg, TURRET_WINDOW_MIN_DEG, TURRET_WINDOW_MAX_DEG);
        double chosenTurretRobotDeg = clamp(turretRobotDeg, TURRET_WINDOW_MIN_DEG, TURRET_WINDOW_MAX_DEG);

        // --- Measured turret angle from tracked ticks ---
        double currentTurretDeg = getTrackedTurretAngleDeg();

        // --- Closed-loop motor control on turret angle ---
        double errorDeg = chosenTurretRobotDeg - currentTurretDeg;
        integral += errorDeg * dt;
        double derivative = (dt > 1e-6) ? (errorDeg - lastError) / dt : 0.0;
        lastError = errorDeg;

        double output = kP * errorDeg + kI * integral + kD * derivative;

        if (Math.abs(errorDeg) > ANGLE_TOLERANCE_DEG) {
            output += Math.signum(errorDeg) * kStatic;
        }

        output = clamp(output, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(output);

        // Telemetry latches
        lastTargetFieldDeg = targetFieldDeg;
        lastRobotFieldHeadingDeg = robotFieldHeadingDeg;
        lastTurretRobotDegCmd = chosenTurretRobotDeg;
        lastTurretRobotDegMeasured = currentTurretDeg;
        lastPowerCmd = output;
    }

    /**
     * Initialize the motor encoder tick frame from the analog absolute reading.
     * This only recovers motor phase within one motor revolution.
     */
    public void initializeMotorTicksFromAbsolute() {
        double motorAbsDeg = getMotorAbsDeg();
        int estimatedMotorTicks = motorDegToTicks(motorAbsDeg);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // We cannot directly set encoder ticks on REV motors, so treat the current zero
        // as equivalent to the current absolute motor phase by storing it in homeMotorTicks.
        // Before hard-stop homing, tracked turret angle is only approximate modulo 120 deg.
        homeMotorTicks = -estimatedMotorTicks;
    }

    /**
     * Homing routine:
     * drive slowly into hard stop, detect sustained over-current while nearly stopped,
     * then define that position as HOME_TURRET_ANGLE_DEG.
     */
    private void runHoming() {
        double current = turretMotor.getCurrent(CurrentUnit.AMPS);
        double vel = turretMotor.getVelocity();
        long now = System.currentTimeMillis();

        turretMotor.setPower(HOMING_POWER);
        lastPowerCmd = HOMING_POWER;

        boolean overCurrent = current >= HOMING_CURRENT_AMPS;
        boolean nearlyStopped = Math.abs(vel) <= HOMING_VEL_TICKS_PER_SEC;

        if (overCurrent && nearlyStopped) {
            if (overCurrentStartMs < 0) {
                overCurrentStartMs = now;
            }

            if (now - overCurrentStartMs >= HOMING_STALL_TIME_MS) {
                turretMotor.setPower(0.0);

                // Define this encoder position as the home reference.
                int currentTicks = getMotorTicksRaw();
                int desiredHomeTicks = turretAngleDegToMotorTicks(HOME_TURRET_ANGLE_DEG);

                // homeMotorTicks is the raw encoder tick value corresponding to HOME_TURRET_ANGLE_DEG
                homeMotorTicks = currentTicks - desiredHomeTicks;
                homed = true;

                // Optional backoff away from hard stop
                if (Math.abs(BACKOFF_DEG_AFTER_HOME) > 1e-6) {
                    setTurretTargetAngleDeg(HOME_TURRET_ANGLE_DEG + BACKOFF_DEG_AFTER_HOME);
                }
            }
        } else {
            overCurrentStartMs = -1;
        }
    }

    /**
     * Directly set a turret target angle in robot-relative degrees.
     * Useful for manual tests or post-home backoff.
     */
    public void setTurretTargetAngleDeg(double deg) {
        lastTurretRobotDegCmd = clamp(deg, TURRET_WINDOW_MIN_DEG, TURRET_WINDOW_MAX_DEG);
    }

    public void updateManualTargetOnly() {
        if (!enabled || (USE_HOMING && !homed)) return;

        double currentTurretDeg = getTrackedTurretAngleDeg();
        double errorDeg = lastTurretRobotDegCmd - currentTurretDeg;

        double output = kP * errorDeg;
        if (Math.abs(errorDeg) > ANGLE_TOLERANCE_DEG) {
            output += Math.signum(errorDeg) * kStatic;
        }

        output = clamp(output, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(output);
        lastTurretRobotDegMeasured = currentTurretDeg;
        lastPowerCmd = output;
    }

    // ===================== Position helpers =====================

    /**
     * Motor-shaft absolute angle from Melonbotics analog output.
     * Returns 0..360.
     */
    public double getMotorAbsDeg() {
        double deg = turretAbs.getVoltage() / ABS_MAX_VOLTAGE * 360.0 + MOTOR_ABS_OFFSET_DEG;
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    /**
     * Raw motor ticks from the encoder port read through the motor object.
     */
    public int getMotorTicksRaw() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Motor ticks relative to the homed/reference frame.
     */
    public int getMotorTicksReferenced() {
        return getMotorTicksRaw() - homeMotorTicks;
    }

    /**
     * Tracked turret angle from motor ticks and gear ratio.
     * This is the main runtime position estimate after homing.
     */
    public double getTrackedTurretAngleDeg() {
        double motorRevs = getMotorTicksReferenced() / TICKS_PER_MOTOR_REV;
        double turretRevs = motorRevs / MOTOR_REVS_PER_TURRET_REV;
        return turretRevs * 360.0;
    }

    public int motorDegToTicks(double motorDeg) {
        return (int) Math.round((motorDeg / 360.0) * TICKS_PER_MOTOR_REV);
    }

    public int turretAngleDegToMotorTicks(double turretDeg) {
        double motorRevs = (turretDeg / 360.0) * MOTOR_REVS_PER_TURRET_REV;
        return (int) Math.round(motorRevs * TICKS_PER_MOTOR_REV);
    }

    public double getMotorCurrentAmps() {
        return turretMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getLastPowerCmd() {
        return lastPowerCmd;
    }

    public double getTargetFieldDeg() {
        return lastTargetFieldDeg;
    }

    public double getRobotFieldHeadingDeg() {
        return lastRobotFieldHeadingDeg;
    }

    public double getTurretRobotDegCmd() {
        return lastTurretRobotDegCmd;
    }

    public double getTurretRobotDegMeasured() {
        return lastTurretRobotDegMeasured;
    }

    public double getLastMotorAbsDeg() {
        return lastMotorAbsDeg;
    }

    // ===================== Utility math =====================

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