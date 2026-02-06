package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.teleOp.competition.MainTeleOp.FORCE_SHOOT_ALL_ZONES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;

import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;

/**
 * Dashboard-only tester:
 * - Shows hub status
 * - Shows RAW + STABLE zone snapshots
 * - Shows each sensor (A/B) per zone
 * - Runs ShotOrderPlanner with a cipher selectable in Dashboard
 * - Displays "Kicking Order: Z1, Z2, Z3" at the top of dashboard telemetry
 *
 * IMPORTANT:
 * - HUB_NAME_A / HUB_NAME_B must match Robot Controller configuration names
 */
@Config
@TeleOp(name = "CZ Detect + Planner Tester (Dashboard)", group = "Test")
public class ColourZoneDetectionPlannerTester extends LinearOpMode {

    // RC Config names
    public static String HUB_NAME_A = "srsHubIndexer";
    public static String HUB_NAME_B = "srsHubPlate";

    // Select in Dashboard
    public static ShotOrderPlanner.Cipher CIPHER = ShotOrderPlanner.Cipher.PPG;

    // Toggle detail sections in Dashboard
    public static boolean SHOW_RAW = true;
    public static boolean SHOW_STABLE = true;
    public static boolean SHOW_EACH_SENSOR = true;

    // For timeline display (purely informational)
    public static double KICK_UP_TIME_S = 0.18;
    public static double RESET_TIME_S = 0.12;

    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.setTelemetryTransmissionInterval(25);

        ColourZoneDetection czd = new ColourZoneDetection(hardwareMap, HUB_NAME_A, HUB_NAME_B);
        ShotOrderPlanner planner = new ShotOrderPlanner();

        waitForStart();

        while (opModeIsActive()) {
            czd.update();

            ColourZoneDetection.Snapshot raw = czd.getRawSnapshot();
            ColourZoneDetection.Snapshot st  = czd.getStableSnapshot();

            // Plan off STABLE snapshot
            List<ShotOrderPlanner.PlannedShot> plan = plan = planner.plan(CIPHER, st, FORCE_SHOOT_ALL_ZONES);

            TelemetryPacket p = new TelemetryPacket();

            // ===== TOP SUMMARY (shows at top of dashboard telemetry) =====
            p.put("Kicking Order", formatKickOrder(plan));
            p.put("Kick Timeline", formatKickTimeline(plan, KICK_UP_TIME_S, RESET_TIME_S));
            p.put("Cipher", CIPHER.toString());
            p.put("Plan Size", plan.size());

            // ===== Hub status =====
            p.put("hubA.ready", czd.hubAReady());
            p.put("hubA.disconnected", czd.hubADisconnected());
            p.put("hubB.ready", czd.hubBReady());
            p.put("hubB.disconnected", czd.hubBDisconnected());

            // ===== Zone snapshots =====
            if (SHOW_RAW) {
                putZoneCombined(p, "z1.raw", raw.z1);
                putZoneCombined(p, "z2.raw", raw.z2);
                putZoneCombined(p, "z3.raw", raw.z3);
            }
            if (SHOW_STABLE) {
                putZoneCombined(p, "z1.st", st.z1);
                putZoneCombined(p, "z2.st", st.z2);
                putZoneCombined(p, "z3.st", st.z3);
            }

            // ===== Each sensor A/B per zone =====
            if (SHOW_EACH_SENSOR) {
                putSensor(p, "z1.a", czd.getZ1A());
                putSensor(p, "z1.b", czd.getZ1B());
                putSensor(p, "z2.a", czd.getZ2A());
                putSensor(p, "z2.b", czd.getZ2B());
                putSensor(p, "z3.a", czd.getZ3A());
                putSensor(p, "z3.b", czd.getZ3B());
            }

            // ===== Planner detail (up to 3 shots) =====
            putPlan(p, plan);

            dash.sendTelemetryPacket(p);

            // ===== Driver Station summary =====
            telemetry.addData("Kicking Order", formatKickOrder(plan));
            telemetry.addData("Kick Timeline", formatKickTimeline(plan, KICK_UP_TIME_S, RESET_TIME_S));
            telemetry.addData("Cipher", CIPHER);
            telemetry.addData("Z1 ST", "%s has=%s prox=%d", st.z1.color, st.z1.hasBall, (st.z1.prox & 0xFFFF));
            telemetry.addData("Z2 ST", "%s has=%s prox=%d", st.z2.color, st.z2.hasBall, (st.z2.prox & 0xFFFF));
            telemetry.addData("Z3 ST", "%s has=%s prox=%d", st.z3.color, st.z3.hasBall, (st.z3.prox & 0xFFFF));
            telemetry.update();

            sleep(20);
        }
    }

    // ====================== Dashboard formatting helpers ======================

    private static String formatKickOrder(List<ShotOrderPlanner.PlannedShot> plan) {
        if (plan == null || plan.isEmpty()) return "Kicking Order: (empty)";
        String zones = plan.stream()
                .map(s -> s.getZone().toString())
                .collect(Collectors.joining(", "));
        return "Kicking Order: " + zones;
    }

    private static String formatKickTimeline(List<ShotOrderPlanner.PlannedShot> plan, double kickUpTimeS, double resetTimeS) {
        if (plan == null || plan.isEmpty()) return "Timeline: (empty)";
        StringBuilder sb = new StringBuilder("Timeline: ");
        double t = 0.0;
        for (int i = 0; i < plan.size(); i++) {
            ShotOrderPlanner.PlannedShot s = plan.get(i);
            sb.append(String.format(Locale.US, "t=%.2fs %s", t, s.getZone()));
            if (i < plan.size() - 1) sb.append(" | ");
            t += (kickUpTimeS + resetTimeS);
        }
        return sb.toString();
    }

    private static void putPlan(TelemetryPacket p, List<ShotOrderPlanner.PlannedShot> plan) {
        p.put("planner.count", plan.size());

        for (int i = 0; i < 3; i++) {
            String key = "planner.s" + (i + 1);
            if (i < plan.size()) {
                ShotOrderPlanner.PlannedShot s = plan.get(i);
                p.put(key + ".zone", s.getZone().toString());
                p.put(key + ".color", s.getColor().toString());
                p.put(key + ".zone_int", zoneToInt(s.getZone()));
                p.put(key + ".color_int", ColourZoneDetection.colorToInt(s.getColor()));
            } else {
                p.put(key + ".zone", "NONE");
                p.put(key + ".color", "NONE");
                p.put(key + ".zone_int", 0);
                p.put(key + ".color_int", 0);
            }
        }
    }

    // ====================== Snapshot telemetry helpers ======================

    private static void putZoneCombined(TelemetryPacket p, String prefix, ColourZoneDetection.ZoneState z) {
        p.put(prefix + ".hasBall", z.hasBall);
        p.put(prefix + ".hasBall_int", z.hasBall ? 1 : 0);
        p.put(prefix + ".color", z.color.toString());
        p.put(prefix + ".color_int", ColourZoneDetection.colorToInt(z.color));

        // chosen sensor values used by the zone combiner
        p.put(prefix + ".prox", (z.prox & 0xFFFF));
        p.put(prefix + ".r", z.r);
        p.put(prefix + ".g", z.g);
        p.put(prefix + ".b", z.b);

        // derived tuning helpers
        p.put(prefix + ".rgb_sum", z.r + z.g + z.b);
        p.put(prefix + ".gb_sum", z.g + z.b);
        p.put(prefix + ".g_minus_r", z.g - z.r);
        p.put(prefix + ".g_minus_b", z.g - z.b);
    }

    private static void putSensor(TelemetryPacket p, String prefix, ColourZoneDetection.SensorState s) {
        p.put(prefix + ".present", s.present);
        p.put(prefix + ".present_int", s.present ? 1 : 0);
        p.put(prefix + ".color", s.color.toString());
        p.put(prefix + ".color_int", ColourZoneDetection.colorToInt(s.color));
        p.put(prefix + ".prox", (s.prox & 0xFFFF));
        p.put(prefix + ".r", s.r);
        p.put(prefix + ".g", s.g);
        p.put(prefix + ".b", s.b);
        p.put(prefix + ".rgb_sum", s.r + s.g + s.b);
    }

    private static int zoneToInt(ColourZoneDetection.ZoneId z) {
        switch (z) {
            case Z1: return 1;
            case Z2: return 2;
            case Z3: return 3;
            default: return 0;
        }
    }
}
