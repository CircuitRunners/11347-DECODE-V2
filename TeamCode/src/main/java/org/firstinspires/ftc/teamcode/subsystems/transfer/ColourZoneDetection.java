package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ColourZoneDetection extends SubsystemBase {

    public enum BallColor { PURPLE, GREEN, NONE }
    public enum ZoneId { Z1, Z2, Z3 }

    /** Per-sensor raw state */
    public static class SensorState {
        public final String cls;        // actual runtime class name
        public final float r, g, b, a;  // normalized
        public final float sum;         // r+g+b
        public final int strength;      // scaled strength used for tie-break
        public final boolean present;
        public final BallColor color;

        public SensorState(String cls, float r, float g, float b, float a,
                           float sum, int strength, boolean present, BallColor color) {
            this.cls = cls;
            this.r = r; this.g = g; this.b = b; this.a = a;
            this.sum = sum;
            this.strength = strength;
            this.present = present;
            this.color = color;
        }
    }

    /** Zone-combined result (chosen “best” sensor when 2 exist). */
    public static class ZoneState {
        public final ZoneId zone;
        public final boolean hasBall;
        public final BallColor color;

        // debug: chosen sensor’s raw values
        public final String cls;
        public final float r, g, b, a;
        public final float sum;
        public final int strength;

        public ZoneState(ZoneId zone, boolean hasBall, BallColor color,
                         String cls, float r, float g, float b, float a, float sum, int strength) {
            this.zone = zone;
            this.hasBall = hasBall;
            this.color = color;
            this.cls = cls;
            this.r = r; this.g = g; this.b = b; this.a = a;
            this.sum = sum;
            this.strength = strength;
        }
    }

    public static class Snapshot {
        public final ZoneState z1, z2, z3;
        public Snapshot(ZoneState z1, ZoneState z2, ZoneState z3) {
            this.z1 = z1; this.z2 = z2; this.z3 = z3;
        }
    }

    // ===================== Tunables =====================
    public static float PRESENT_MIN_A_Z1 = 0.02f;
    public static float PRESENT_MIN_A_Z2 = 0.02f;
    public static float PRESENT_MIN_A_Z3 = 0.02f;

    public static float PRESENT_MIN_SUM_Z1 = 0.02f;
    public static float PRESENT_MIN_SUM_Z2 = 0.02f;
    public static float PRESENT_MIN_SUM_Z3 = 0.02f;

    public static int DEBOUNCE_MS = 120;

    // Prefer ratio-based tests (more robust across different sensors / gains)
    public static float GREEN_RATIO_MIN = 1.35f;   // g must be >= max(r,b) * this
    public static float PURPLE_RB_RATIO_MIN = 1.15f; // (r+b) must be >= g * this
    public static float PURPLE_BLUE_SHARE_MIN = 0.28f; // b/sum minimum (tune)
    public static float BRIGHTNESS_MIN_WHEN_PRESENT = 0.02f; // sum gate for classification

    // Strength scaling (tie-breaker only)
    public static float STRENGTH_SCALE = 1000.0f;

    // Optional: set gain on all sensors (can help a lot)
    public static float SENSOR_GAIN = 2.0f;
    public static boolean SET_GAIN_EACH_LOOP = false;

    // ===================== Hardware =====================
    private final NormalizedColorSensor z1aC, z2aC, z3aC;
    private final NormalizedColorSensor z1bC, z2bC, z3bC;

    private final ElapsedTime clock = new ElapsedTime();

    private Snapshot raw;
    private Snapshot stable;

    private final Candidate c1 = new Candidate();
    private final Candidate c2 = new Candidate();
    private final Candidate c3 = new Candidate();

    private static class Candidate {
        boolean hasBall;
        BallColor color;
        long sinceMs;
        boolean active;
        void set(boolean hasBall, BallColor color, long nowMs) {
            this.hasBall = hasBall;
            this.color = color;
            this.sinceMs = nowMs;
            this.active = true;
        }
    }

    public ColourZoneDetection(HardwareMap hw,
                               String z1aName, String z2aName, String z3aName,
                               String z1bName, String z2bName, String z3bName) {

        z1aC = hw.get(NormalizedColorSensor.class, z1aName);
        z2aC = hw.get(NormalizedColorSensor.class, z2aName);
        z3aC = hw.get(NormalizedColorSensor.class, z3aName);

        z1bC = hw.get(NormalizedColorSensor.class, z1bName);
        z2bC = hw.get(NormalizedColorSensor.class, z2bName);
        z3bC = hw.get(NormalizedColorSensor.class, z3bName);

        // One-time gain set (safe for both V3 + Color/Range)
        safeSetGain(z1aC); safeSetGain(z2aC); safeSetGain(z3aC);
        safeSetGain(z1bC); safeSetGain(z2bC); safeSetGain(z3bC);

        // init snapshots
        raw = new Snapshot(
                emptyZone(ZoneId.Z1),
                emptyZone(ZoneId.Z2),
                emptyZone(ZoneId.Z3)
        );
        stable = raw;

        clock.reset();
    }

    @Override
    public void periodic() {
        update();
    }

    /** Call once per loop. */
    public void update() {
        if (SET_GAIN_EACH_LOOP) {
            safeSetGain(z1aC); safeSetGain(z2aC); safeSetGain(z3aC);
            safeSetGain(z1bC); safeSetGain(z2bC); safeSetGain(z3bC);
        }

        long nowMs = (long) clock.milliseconds();

        raw = new Snapshot(
                computeZoneNow(ZoneId.Z1, z1aC, z1bC, PRESENT_MIN_A_Z1, PRESENT_MIN_SUM_Z1),
                computeZoneNow(ZoneId.Z2, z2aC, z2bC, PRESENT_MIN_A_Z2, PRESENT_MIN_SUM_Z2),
                computeZoneNow(ZoneId.Z3, z3aC, z3bC, PRESENT_MIN_A_Z3, PRESENT_MIN_SUM_Z3)
        );

        stable = new Snapshot(
                debounce(raw.z1, c1, stable.z1, nowMs),
                debounce(raw.z2, c2, stable.z2, nowMs),
                debounce(raw.z3, c3, stable.z3, nowMs)
        );
    }

    public Snapshot getRawSnapshot() { return raw; }
    public Snapshot getStableSnapshot() { return stable; }

    // Raw per-sensor accessors
    public SensorState getZ1A() { return evalSensor(z1aC, PRESENT_MIN_A_Z1, PRESENT_MIN_SUM_Z1); }
    public SensorState getZ1B() { return evalSensor(z1bC, PRESENT_MIN_A_Z1, PRESENT_MIN_SUM_Z1); }
    public SensorState getZ2A() { return evalSensor(z2aC, PRESENT_MIN_A_Z2, PRESENT_MIN_SUM_Z2); }
    public SensorState getZ2B() { return evalSensor(z2bC, PRESENT_MIN_A_Z2, PRESENT_MIN_SUM_Z2); }
    public SensorState getZ3A() { return evalSensor(z3aC, PRESENT_MIN_A_Z3, PRESENT_MIN_SUM_Z3); }
    public SensorState getZ3B() { return evalSensor(z3bC, PRESENT_MIN_A_Z3, PRESENT_MIN_SUM_Z3); }

    private ZoneState debounce(ZoneState computed, Candidate cand, ZoneState stableState, long nowMs) {
        boolean same = computed.hasBall == stableState.hasBall && computed.color == stableState.color;
        if (same) { cand.active = false; return computed; }

        if (!cand.active) { cand.set(computed.hasBall, computed.color, nowMs); return stableState; }

        boolean candMatches = cand.hasBall == computed.hasBall && cand.color == computed.color;
        if (!candMatches) { cand.set(computed.hasBall, computed.color, nowMs); return stableState; }

        if (nowMs - cand.sinceMs >= DEBOUNCE_MS) { cand.active = false; return computed; }

        return stableState;
    }

    private ZoneState computeZoneNow(ZoneId zone,
                                     NormalizedColorSensor aC,
                                     NormalizedColorSensor bC,
                                     float presentMinA,
                                     float presentMinSum) {

        SensorState a = evalSensor(aC, presentMinA, presentMinSum);
        SensorState b = evalSensor(bC, presentMinA, presentMinSum);

        if (!a.present && !b.present) return emptyZone(zone);

        if (a.present && !b.present) return zoneFromSensor(zone, a);
        if (!a.present && b.present) return zoneFromSensor(zone, b);

        // both present: prefer matching color; otherwise prefer stronger; otherwise prefer non-NONE
        if (a.color == b.color) {
            return zoneFromSensor(zone, (a.strength >= b.strength) ? a : b, a.color);
        }

        if (a.color == BallColor.NONE && b.color != BallColor.NONE) return zoneFromSensor(zone, b);
        if (b.color == BallColor.NONE && a.color != BallColor.NONE) return zoneFromSensor(zone, a);

        return zoneFromSensor(zone, (a.strength >= b.strength) ? a : b);
    }

    private static ZoneState emptyZone(ZoneId zone) {
        return new ZoneState(zone, false, BallColor.NONE, "none",
                0, 0, 0, 0, 0, 0);
    }

    private static ZoneState zoneFromSensor(ZoneId zone, SensorState s) {
        return new ZoneState(zone, true, s.color, s.cls, s.r, s.g, s.b, s.a, s.sum, s.strength);
    }

    private static ZoneState zoneFromSensor(ZoneId zone, SensorState s, BallColor forced) {
        return new ZoneState(zone, true, forced, s.cls, s.r, s.g, s.b, s.a, s.sum, s.strength);
    }

    private SensorState evalSensor(NormalizedColorSensor c, float presentMinA, float presentMinSum) {
        if (c == null) {
            return new SensorState("null", 0,0,0,0, 0, 0, false, BallColor.NONE);
        }

        NormalizedRGBA rgba = c.getNormalizedColors();
        float r = rgba.red;
        float g = rgba.green;
        float b = rgba.blue;
        float a = rgba.alpha;
        float sum = r + g + b;

        boolean present = (a >= presentMinA) && (sum >= presentMinSum);

        BallColor color = classify(present, r, g, b, sum);

        int strength = (int) (STRENGTH_SCALE * (a + sum));

        return new SensorState(c.getClass().getSimpleName(), r, g, b, a, sum, strength, present, color);
    }

    private BallColor classify(boolean present, float r, float g, float b, float sum) {
        if (!present) return BallColor.NONE;
        if (sum < BRIGHTNESS_MIN_WHEN_PRESENT) return BallColor.NONE;

        float maxRB = Math.max(r, b);
        if (g >= maxRB * GREEN_RATIO_MIN) return BallColor.GREEN;

        // “purple” tends to be high in (r+b) relative to g, and b should be a decent share of sum
        float rb = r + b;
        float blueShare = (sum > 1e-6f) ? (b / sum) : 0.0f;

        if (rb >= g * PURPLE_RB_RATIO_MIN && blueShare >= PURPLE_BLUE_SHARE_MIN) {
            return BallColor.PURPLE;
        }

        return BallColor.NONE;
    }

    private static void safeSetGain(NormalizedColorSensor s) {
        if (s == null) return;
        try {
            s.setGain(SENSOR_GAIN);
        } catch (Exception ignored) {
            // some implementations may not support gain; ignore safely
        }
    }

    public static int colorToInt(BallColor c) {
        switch (c) {
            case PURPLE: return 1;
            case GREEN:  return 2;
            default:     return 0;
        }
    }
}
