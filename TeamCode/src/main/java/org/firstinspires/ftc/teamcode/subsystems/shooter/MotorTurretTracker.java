package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class MotorTurretTracker extends SubsystemBase {

    // ===================== CONTROLLER =====================
    private final PIDController controller;
    public static double p = 0.00075;
    public static double i = 0.0;
    public static double d = 0.00003;

    // Static friction compensation
    public static double f = 0.0065;
    public static int staticFrictionDeadbandTicks = 20;

    private int target = 0;

    // ===================== DEVICES =====================
    private final DcMotorEx turret;
    private final DigitalChannel magneticLimitSwitch;

    // ===================== STATES =====================
    private boolean enabled = false;
    private boolean homed = false;
    private final boolean isRed;

    // ===================== Turret geometry / gearing =====================
    public static double MOTOR_REVS_PER_TURRET_REV = 3.0;
    public static double TICKS_PER_MOTOR_REV = 4096.0; // Melonbotics CPR

    // Relative encoder home = 0 at startup magnet
    // Red: lower/home side
    // Blue: upper/home side
    public static int LOWER_LIMIT = -200;
    public static int UPPER_LIMIT = 12200;

    // These are turret angles measured FROM THE HOME POSITION for each alliance.
    // Keep them positive. Tick sign is handled separately.
    public static double TURRET_WINDOW_MIN_DEG = 0.0;
    public static double TURRET_WINDOW_MAX_DEG = 355.0;

    // Robot-relative heading that corresponds to turret HOME for each alliance.
    // Tune these on-robot.
    //
    // Example meaning:
    // If at RED startup the turret is physically at its home magnet and that points
    // robot-relative -135 deg, then RED_ZERO_REFERENCE_DEG should be -135.
    //
    // If at BLUE startup the turret home magnet points robot-relative +45 deg,
    // then BLUE_ZERO_REFERENCE_DEG should be +45.
    public static double RED_ZERO_REFERENCE_DEG = -135.0;
    public static double BLUE_ZERO_REFERENCE_DEG = 45.0;

    // Fine trim after reference conversion
    public static double TURRET_TRIM_DEG = 0.0;

    // Odom / pivot adjustments
    public static boolean SWAP_XY = false;
    public static boolean INVERT_X = false;
    public static boolean INVERT_Y = false;
    public static double HEADING_OFFSET_DEG = 0.0;
    public static double TURRET_OFFSET_X_IN = 0.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    // Target point in field coordinates
    private double targetFieldX = 0.0;
    private double targetFieldY = 0.0;

    // Telemetry latches
    private double lastTargetFieldDeg = 0.0;
    private double lastRobotFieldHeadingDeg = 0.0;
    private double lastTurretRobotDegRaw = 0.0;
    private double lastTurretHomeFrameDegCmd = 0.0;
    private double lastTurretHomeFrameDegMeasured = 0.0;
    private double lastPower = 0.0;

    public MotorTurretTracker(HardwareMap hardwareMap, boolean isRed) {
        this.isRed = isRed;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        magneticLimitSwitch = hardwareMap.get(DigitalChannel.class, "magLimit");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        controller = new PIDController(p, i, d);
        target = 0;
        homed = false;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            turret.setPower(0);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isHomed() {
        return homed;
    }

    @Override
    public void periodic() {
        if (!enabled || !homed) {
            turret.setPower(0);
            lastPower = 0;
            return;
        }

        controller.setPID(p, i, d);

        int currentPos = turret.getCurrentPosition();
        double pid = controller.calculate(currentPos, target);

        int error = target - currentPos;
        double ff = 0.0;
        if (Math.abs(error) > staticFrictionDeadbandTicks) {
            ff = Math.signum(error) * f;
        }

        double power = pid + ff;

        // Keep this sign the same as your working version.
        turret.setPower(-power);

        lastPower = power;
        lastTurretHomeFrameDegMeasured = getCurrentTurretDegFromHome();
    }

    // Only used at startup to confirm turret is sitting on the alliance home magnet
    public boolean isAtLimit() {
        return !magneticLimitSwitch.getState();
    }

    // Call this once at startup when the turret is physically at the alliance's home limit
    public void zeroAtStartupIfOnLimit() {
        if (isAtLimit()) {
            reset();
            homed = true;
        } else {
            homed = false;
        }
    }

    public void forceHome() {
        reset();
        homed = true;
    }

    public int getCurrentEncoderPos() {
        return turret.getCurrentPosition();
    }

    public void setTargetTicks(int targetTicks) {
        target = targetTicks;
    }

    public int getTargetTicks() {
        return target;
    }

    public void reset() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = 0;
    }

    public double getCurrent() {
        return turret.getCurrent(CurrentUnit.AMPS);
    }

    public double getTicksPerTurretDegree() {
        return (TICKS_PER_MOTOR_REV * MOTOR_REVS_PER_TURRET_REV) / 360.0;
    }

    // ===================== Angle / tick conversion =====================

    // Turret angle measured from alliance home position.
    // Positive for both alliances.
    public double getCurrentTurretDegFromHome() {
        double ticksPerDeg = getTicksPerTurretDegree();

        if (isRed) {
            return turret.getCurrentPosition() / ticksPerDeg;
        } else {
            return -turret.getCurrentPosition() / ticksPerDeg;
        }
    }

    // Convert "home-frame turret angle" into encoder ticks.
    // Red moves positive from home, blue moves negative from home.
    public int angleDegToTicks(double turretDegFromHome) {
        double clampedDeg = clamp(turretDegFromHome, TURRET_WINDOW_MIN_DEG, TURRET_WINDOW_MAX_DEG);
        double ticksPerDeg = getTicksPerTurretDegree();

        if (isRed) {
            return (int) Math.round(clampedDeg * ticksPerDeg);
        } else {
            return (int) Math.round(-clampedDeg * ticksPerDeg);
        }
    }

    public void setTargetDegFromHome(double turretDegFromHome) {
        double clampedDeg = clamp(turretDegFromHome, TURRET_WINDOW_MIN_DEG, TURRET_WINDOW_MAX_DEG);
        setTargetTicks(angleDegToTicks(clampedDeg));
        lastTurretHomeFrameDegCmd = clampedDeg;
    }

    public void setTargetFieldPointInches(double x, double y) {
        targetFieldX = x;
        targetFieldY = y;
    }

    private double getAllianceZeroReferenceDeg() {
        return isRed ? RED_ZERO_REFERENCE_DEG : BLUE_ZERO_REFERENCE_DEG;
    }

    /**
     * Robot-relative aim angle -> turret-home-frame angle
     *
     * Example:
     * If raw robot-relative target is -135 deg and RED home physically points at -135 deg,
     * then home-frame command should be 0 deg.
     */
    private double robotRelativeToHomeFrameDeg(double robotRelativeDeg) {
        double zeroRef = getAllianceZeroReferenceDeg();

        double homeFrameDeg = robotRelativeDeg - zeroRef;
        homeFrameDeg = wrap0to360(homeFrameDeg);

        // choose the representation closest to the allowed window
        homeFrameDeg = chooseClosestToWindow(homeFrameDeg, TURRET_WINDOW_MIN_DEG, TURRET_WINDOW_MAX_DEG);

        return clamp(homeFrameDeg, TURRET_WINDOW_MIN_DEG, TURRET_WINDOW_MAX_DEG);
    }

    // ===================== Main aiming update =====================

    public void updateAim(Pose2D robotPose) {
        if (!enabled || !homed) return;

        double rx = robotPose.getX(DistanceUnit.INCH);
        double ry = robotPose.getY(DistanceUnit.INCH);

        if (SWAP_XY) {
            double tmp = rx;
            rx = ry;
            ry = tmp;
        }
        if (INVERT_X) rx = -rx;
        if (INVERT_Y) ry = -ry;

        double robotHeadingRad = robotPose.getHeading(AngleUnit.RADIANS) + Math.toRadians(HEADING_OFFSET_DEG);
        double robotFieldHeadingDeg = wrap0to360(Math.toDegrees(robotHeadingRad));

        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);

        double pivotX = rx + (TURRET_OFFSET_X_IN * cosH - TURRET_OFFSET_Y_IN * sinH);
        double pivotY = ry + (TURRET_OFFSET_X_IN * sinH + TURRET_OFFSET_Y_IN * cosH);

        double targetFieldDeg = wrap0to360(Math.toDegrees(Math.atan2(targetFieldY - pivotY, targetFieldX - pivotX)));

        // Standard robot-relative bearing to target
        double turretRobotDegRaw = wrapTo180(targetFieldDeg - robotFieldHeadingDeg);

        // Convert into turret's own home-based angle frame
        double turretFromHomeDeg = robotRelativeToHomeFrameDeg(turretRobotDegRaw + TURRET_TRIM_DEG);

        setTargetDegFromHome(turretFromHomeDeg);

        lastTargetFieldDeg = targetFieldDeg;
        lastRobotFieldHeadingDeg = robotFieldHeadingDeg;
        lastTurretRobotDegRaw = turretRobotDegRaw;
    }

    // ===================== Telemetry getters =====================

    public double getLastTargetFieldDeg() {
        return lastTargetFieldDeg;
    }

    public double getLastRobotFieldHeadingDeg() {
        return lastRobotFieldHeadingDeg;
    }

    public double getLastTurretRobotDegRaw() {
        return lastTurretRobotDegRaw;
    }

    public double getLastTurretHomeFrameDegCmd() {
        return lastTurretHomeFrameDegCmd;
    }

    public double getLastTurretHomeFrameDegMeasured() {
        return lastTurretHomeFrameDegMeasured;
    }

    public double getLastPower() {
        return lastPower;
    }

    // ===================== Utility math =====================

    private static double chooseClosestToWindow(double deg, double winMin, double winMax) {
        double best = deg;
        double bestCost = costToWindow(deg, winMin, winMax);

        double d1 = deg + 360.0;
        double c1 = costToWindow(d1, winMin, winMax);
        if (c1 < bestCost) {
            bestCost = c1;
            best = d1;
        }

        double d2 = deg - 360.0;
        double c2 = costToWindow(d2, winMin, winMax);
        if (c2 < bestCost) {
            best = d2;
        }

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