package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Heading tracking from two opposite-side odometry pods (differential heading).
 *
 * dTheta = (dRight - dLeft) / trackWidth
 *
 * "Absolute" here means: heading is consistent over time without IMU drift,
 * but it's still relative to robots start position
 *
 * Units:
 *  * - trackWidthIn is inches (center-to-center of pod wheels, perpendicular to wheel direction)
 *  * - distances are inches
 *  * - heading is radians/degrees
 */
@Config
public class OdoAbsoluteHeadingTracking extends SubsystemBase {
    // goBILDA 4-Bar Pod defaults (32mm wheel, 2000 events/rev)
    public static final double GOBILDA_4BAR_TICKS_PER_MM = 19.89436789;
    public static final double GOBILDA_4BAR_INCHES_PER_TICK =
            1.0 / (GOBILDA_4BAR_TICKS_PER_MM * 25.4);

    private final DcMotorEx leftEncoder;
    private final DcMotorEx rightEncoder;

    public static double trackWidthIn;
    public static boolean leftReversed;
    public static boolean rightReversed;

    private int lastLeftTicks;
    private int lastRightTicks;

    private double headingRad;
    private boolean initialized = false;

    public OdoAbsoluteHeadingTracking(
            DcMotorEx leftEncoder,
            DcMotorEx rightEncoder,
            double trackWidthIn,
            boolean leftReversed,
            boolean rightReversed
    ) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.trackWidthIn = trackWidthIn;
        this.leftReversed = leftReversed;
        this.rightReversed = rightReversed;
    }

    public OdoAbsoluteHeadingTracking(DcMotorEx leftEncoder, DcMotorEx rightEncoder) {
        this(leftEncoder, rightEncoder, 14.841888386312671569085163467971, false, true);
    }

    /** Call once in init(), or any time you want to re-zero heading. */
    public void reset(double headingRad) {
        this.headingRad = wrapRad(headingRad);
        lastLeftTicks = getLeftTicks();
        lastRightTicks = getRightTicks();
        initialized = true;
    }

    /** Call every loop(). */
    public void update() {
        if (!initialized) {
            reset(0.0);
            return;
        }

        int leftTicks = getLeftTicks();
        int rightTicks = getRightTicks();

        int dLeftTicks = leftTicks - lastLeftTicks;
        int dRightTicks = rightTicks - lastRightTicks;

        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;

        double dLeftIN = dLeftTicks * GOBILDA_4BAR_INCHES_PER_TICK;
        double dRightIN = dRightTicks * GOBILDA_4BAR_INCHES_PER_TICK;

        double dTheta = (dRightIN - dLeftIN) / trackWidthIn;
        headingRad = wrapRad(headingRad + dTheta);
    }

    @Override
    public void periodic() {
        update();
    }

    public double getHeadingRad() {
        return headingRad;
    }
    public double getHeadingDeg() {
        return Math.toDegrees(headingRad);
    }

    public void setHeadingRad(double headingRad) {
        this.headingRad = wrapRad(headingRad);
    }

    private int getLeftTicks() {
        int t = leftEncoder.getCurrentPosition();
        return leftReversed ? -t : t;
    }

    private int getRightTicks() {
        int t = rightEncoder.getCurrentPosition();
        return rightReversed ? -t : t;
    }

    private static double wrapRad(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        return rad;
    }
}
