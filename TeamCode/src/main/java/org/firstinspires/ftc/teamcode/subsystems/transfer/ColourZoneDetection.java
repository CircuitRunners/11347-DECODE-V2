package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.support.SRSHub;

@Config
public class ColourZoneDetection extends SubsystemBase {

    public enum BallColor { PURPLE, GREEN, NONE }
    public enum ZoneId { Z1, Z2, Z3 }

    public static class SensorState {
        public final int r, g, b;
        public final short prox;
        public final boolean present;
        public final BallColor color;

        public SensorState(int r, int g, int b, short prox, boolean present, BallColor color) {
            this.r = r; this.g = g; this.b = b; this.prox = prox; this.present = present; this.color = color;
        }
    }

    public static class ZoneState {
        public final ZoneId zone;
        public final boolean hasBall;
        public final BallColor color;

        // debug (chosen sensor that drove decision)
        public final int r, g, b;
        public final short prox;

        public ZoneState(ZoneId zone, boolean hasBall, BallColor color, int r, int g, int b, short prox) {
            this.zone = zone;
            this.hasBall = hasBall;
            this.color = color;
            this.r = r;
            this.g = g;
            this.b = b;
            this.prox = prox;
        }
    }

    public static class Snapshot {
        public final ZoneState z1, z2, z3;
        public Snapshot(ZoneState z1, ZoneState z2, ZoneState z3) { this.z1 = z1; this.z2 = z2; this.z3 = z3; }
    }

    // ======== Tunables ========
    public static int PROX_MIN_Z1 = 120;
    public static int PROX_MIN_Z2 = 120;
    public static int PROX_MIN_Z3 = 120;

    public static int DEBOUNCE_MS = 120;

    public static int GREEN_MARGIN = 60;

    public static int PURPLE_GB_MIN = 260;
    public static int PURPLE_BLUE_MIN = 80;
    public static int PURPLE_R_MULT_NUM = 13;
    public static int PURPLE_R_MULT_DEN = 10;

    public static int RGB_SUM_MIN_WHEN_PRESENT = 120;

    // ======== SRSHubs ========
    private final SRSHub hubA;
    private final SRSHub hubB;

    // One sensor per bus per hub. (A = hubA, B = hubB)
    private SRSHub.APDS9151 z1a, z2a, z3a;
    private SRSHub.APDS9151 z1b, z2b, z3b;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();

    // RAW combined zone state
    private Snapshot raw = new Snapshot(
            new ZoneState(ZoneId.Z1, false, BallColor.NONE, 0, 0, 0, (short) 0),
            new ZoneState(ZoneId.Z2, false, BallColor.NONE, 0, 0, 0, (short) 0),
            new ZoneState(ZoneId.Z3, false, BallColor.NONE, 0, 0, 0, (short) 0)
    );

    // STABLE combined zone state
    private Snapshot stable = raw;

    private final Candidate c1 = new Candidate();
    private final Candidate c2 = new Candidate();
    private final Candidate c3 = new Candidate();

    private static class Candidate {
        boolean hasBall;
        BallColor color;
        long sinceMs;
        boolean active;
        void set(boolean hasBall, BallColor color, long nowMs) {
            this.hasBall = hasBall; this.color = color; this.sinceMs = nowMs; this.active = true;
        }
    }

    /**
     * @param hardwareMap hardware map
     * @param hubNameA RC config name for hub A
     * @param hubNameB RC config name for hub B
     */
    public ColourZoneDetection(HardwareMap hardwareMap, String hubNameA, String hubNameB) {
        SRSHub.Config cfgA = new SRSHub.Config();
        cfgA.addI2CDevice(1, new SRSHub.APDS9151());
        cfgA.addI2CDevice(2, new SRSHub.APDS9151());
        cfgA.addI2CDevice(3, new SRSHub.APDS9151());

        SRSHub.Config cfgB = new SRSHub.Config();
        cfgB.addI2CDevice(1, new SRSHub.APDS9151());
        cfgB.addI2CDevice(2, new SRSHub.APDS9151());
        cfgB.addI2CDevice(3, new SRSHub.APDS9151());

        hubA = hardwareMap.get(SRSHub.class, hubNameA);
        hubB = hardwareMap.get(SRSHub.class, hubNameB);

        hubA.init(cfgA);
        hubB.init(cfgB);

        waitForBothReady(5.0);

        z1a = hubA.getI2CDevice(1, SRSHub.APDS9151.class);
        z2a = hubA.getI2CDevice(2, SRSHub.APDS9151.class);
        z3a = hubA.getI2CDevice(3, SRSHub.APDS9151.class);

        z1b = hubB.getI2CDevice(1, SRSHub.APDS9151.class);
        z2b = hubB.getI2CDevice(2, SRSHub.APDS9151.class);
        z3b = hubB.getI2CDevice(3, SRSHub.APDS9151.class);

        clock.reset();
    }

    private void waitForBothReady(double timeoutSeconds) {
        runtime.reset();
        while (!(hubA.ready() && hubB.ready()) && runtime.seconds() < timeoutSeconds) {
            Thread.yield();
        }
    }

    public boolean hubAReady() { return hubA != null && hubA.ready(); }
    public boolean hubBReady() { return hubB != null && hubB.ready(); }
    public boolean hubADisconnected() { return hubA == null || hubA.disconnected(); }
    public boolean hubBDisconnected() { return hubB == null || hubB.disconnected(); }

    /** Call once per loop */
    public void update() {
//        if (hubA == null || hubB == null) return;
//        if (hubA.disconnected() || hubB.disconnected()) return;

        if (!hubADisconnected()) {
            hubA.update();
        }
        if (!hubBDisconnected()) {
            hubB.update();
        }

        long nowMs = (long) clock.milliseconds();

        raw = new Snapshot(
                computeZoneNow(ZoneId.Z1, z1a, z1b, PROX_MIN_Z1),
                computeZoneNow(ZoneId.Z2, z2a, z2b, PROX_MIN_Z2),
                computeZoneNow(ZoneId.Z3, z3a, z3b, PROX_MIN_Z3)
        );

        stable = new Snapshot(
                debounce(raw.z1, c1, stable.z1, nowMs),
                debounce(raw.z2, c2, stable.z2, nowMs),
                debounce(raw.z3, c3, stable.z3, nowMs)
        );
    }

    public Snapshot getRawSnapshot() {
        return raw;
    }
    public Snapshot getStableSnapshot() {
        return stable;
    }

    // Expose individual sensor states for telemetry
    public SensorState getZ1A() {
        return evalSensor(z1a, PROX_MIN_Z1);
    }
    public SensorState getZ1B() {
        return evalSensor(z1b, PROX_MIN_Z1);
    }
    public SensorState getZ2A() {
        return evalSensor(z2a, PROX_MIN_Z2);
    }
    public SensorState getZ2B() {
        return evalSensor(z2b, PROX_MIN_Z2);
    }
    public SensorState getZ3A() {
        return evalSensor(z3a, PROX_MIN_Z3);
    }
    public SensorState getZ3B() {
        return evalSensor(z3b, PROX_MIN_Z3);
    }

    private ZoneState debounce(ZoneState computed, Candidate cand, ZoneState stableState, long nowMs) {
        boolean same = computed.hasBall == stableState.hasBall && computed.color == stableState.color;
        if (same) { cand.active = false; return computed; }

        if (!cand.active) { cand.set(computed.hasBall, computed.color, nowMs); return stableState; }

        boolean candMatches = cand.hasBall == computed.hasBall && cand.color == computed.color;
        if (!candMatches) { cand.set(computed.hasBall, computed.color, nowMs); return stableState; }

        if (nowMs - cand.sinceMs >= DEBOUNCE_MS) { cand.active = false; return computed; }

        return stableState;
    }

    private static class Eval {
        final SensorState ss;
        final int strength;
        Eval(SensorState ss, int strength) { this.ss = ss; this.strength = strength; }
    }

    private ZoneState computeZoneNow(ZoneId zone, SRSHub.APDS9151 sA, SRSHub.APDS9151 sB, int proxMin) {
        Eval a = evalWithStrength(sA, proxMin);
        Eval b = evalWithStrength(sB, proxMin);

        // neither sees ball
        if (!a.ss.present && !b.ss.present) {
            return new ZoneState(zone, false, BallColor.NONE, 0, 0, 0, (short) 0);
        }

        // one sees ball
        if (a.ss.present && !b.ss.present) {
            return new ZoneState(zone, true, a.ss.color, a.ss.r, a.ss.g, a.ss.b, a.ss.prox);
        }
        if (!a.ss.present && b.ss.present) {
            return new ZoneState(zone, true, b.ss.color, b.ss.r, b.ss.g, b.ss.b, b.ss.prox);
        }

        // both see ball: should match; if mismatch, pick stronger (or non-NONE)
        if (a.ss.color == b.ss.color) {
            Eval pick = (a.strength >= b.strength) ? a : b;
            return new ZoneState(zone, true, a.ss.color, pick.ss.r, pick.ss.g, pick.ss.b, pick.ss.prox);
        }

        if (a.ss.color == BallColor.NONE && b.ss.color != BallColor.NONE) {
            return new ZoneState(zone, true, b.ss.color, b.ss.r, b.ss.g, b.ss.b, b.ss.prox);
        }
        if (b.ss.color == BallColor.NONE && a.ss.color != BallColor.NONE) {
            return new ZoneState(zone, true, a.ss.color, a.ss.r, a.ss.g, a.ss.b, a.ss.prox);
        }

        Eval pick = (a.strength >= b.strength) ? a : b;
        return new ZoneState(zone, true, pick.ss.color, pick.ss.r, pick.ss.g, pick.ss.b, pick.ss.prox);
    }

    private SensorState evalSensor(SRSHub.APDS9151 s, int proxMin) {
        return evalWithStrength(s, proxMin).ss;
    }

    private Eval evalWithStrength(SRSHub.APDS9151 s, int proxMin) {
        if (s == null || s.disconnected) {
            SensorState ss = new SensorState(0, 0, 0, (short) 0, false, BallColor.NONE);
            return new Eval(ss, 0);
        }

        int r = s.red, g = s.green, b = s.blue;
        short p = s.proximity;

        boolean present = (p & 0xFFFF) >= proxMin;
        BallColor c = classify(present, r, g, b);

        SensorState ss = new SensorState(r, g, b, p, present, c);
        int strength = (p & 0xFFFF) + (r + g + b);
        return new Eval(ss, strength);
    }

    private BallColor classify(boolean present, int r, int g, int b) {
        if (!present) return BallColor.NONE;

        int sum = r + g + b;
        if (sum < RGB_SUM_MIN_WHEN_PRESENT) return BallColor.NONE;

        if (g > r + GREEN_MARGIN && g > b + GREEN_MARGIN) return BallColor.GREEN;

        int gb = g + b;
        boolean gbStrong = gb >= PURPLE_GB_MIN;
        boolean bluePresent = b >= PURPLE_BLUE_MIN;
        boolean dominatesR = (gb * PURPLE_R_MULT_DEN) > (r * PURPLE_R_MULT_NUM);

        if (gbStrong && bluePresent && dominatesR) return BallColor.PURPLE;

        return BallColor.NONE;
    }

    public static int colorToInt(BallColor c) {
        switch (c) {
            case PURPLE: return 1;
            case GREEN: return 2;
            default: return 0;
        }
    }
}
