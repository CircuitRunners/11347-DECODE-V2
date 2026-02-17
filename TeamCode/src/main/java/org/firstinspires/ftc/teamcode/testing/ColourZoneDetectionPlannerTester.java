package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;

import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;

@Config
@TeleOp(name = "CZ RAW + Planner Tester (Dashboard)", group = "Test")
public class ColourZoneDetectionPlannerTester extends LinearOpMode {

    // Names in RC config
    public static ShotOrderPlanner.Cipher CIPHER = ShotOrderPlanner.Cipher.PPG;

    public static boolean SHOW_RAW_ZONES = true;
    public static boolean SHOW_STABLE_ZONES = true;
    public static boolean SHOW_EACH_SENSOR = true;

    public static double KICK_UP_TIME_S = 0.18;
    public static double RESET_TIME_S = 0.12;

    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.setTelemetryTransmissionInterval(25);

        ColourZoneDetection czd;
        try {
            czd = new ColourZoneDetection(hardwareMap, "z1CSa", "z2CSa", "z3CSa",
                    "z1CSb", "z2CSb", "z3CSb");
        } catch (Exception e) {
            telemetry.addLine("CZ init FAILED (check RC names / wiring)");
            telemetry.addData("EX", e.toString());
            telemetry.update();
            sleep(8000);
            return;
        }

        ShotOrderPlanner planner = new ShotOrderPlanner();

        telemetry.addLine("CZ init OK");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            czd.update();

            ColourZoneDetection.Snapshot raw = czd.getRawSnapshot();
            ColourZoneDetection.Snapshot st  = czd.getStableSnapshot();

            List<ShotOrderPlanner.PlannedShot> plan = planner.plan(CIPHER, st, false);

            TelemetryPacket p = new TelemetryPacket();
            p.put("Kicking Order", formatKickOrder(plan));
            p.put("Kick Timeline", formatKickTimeline(plan, KICK_UP_TIME_S, RESET_TIME_S));
            p.put("Cipher", CIPHER.toString());
            p.put("Plan Size", plan.size());

            if (SHOW_RAW_ZONES) {
                putZone(p, "z1.raw", raw.z1);
                putZone(p, "z2.raw", raw.z2);
                putZone(p, "z3.raw", raw.z3);
            }
            if (SHOW_STABLE_ZONES) {
                putZone(p, "z1.st", st.z1);
                putZone(p, "z2.st", st.z2);
                putZone(p, "z3.st", st.z3);
            }

            if (SHOW_EACH_SENSOR) {
                putSensor(p, "z1.a", czd.getZ1A());
                putSensor(p, "z1.b", czd.getZ1B());
                putSensor(p, "z2.a", czd.getZ2A());
                putSensor(p, "z2.b", czd.getZ2B());
                putSensor(p, "z3.a", czd.getZ3A());
                putSensor(p, "z3.b", czd.getZ3B());
            }

            dash.sendTelemetryPacket(p);

            telemetry.addData("Kicking Order", formatKickOrder(plan));
            telemetry.addData("Cipher", CIPHER);

            telemetry.addData("Z1 ST", "has=%s col=%s sum=%.4f a=%.4f cls=%s",
                    st.z1.hasBall, st.z1.color, st.z1.sum, st.z1.a, st.z1.cls);
            telemetry.addData("Z2 ST", "has=%s col=%s sum=%.4f a=%.4f cls=%s",
                    st.z2.hasBall, st.z2.color, st.z2.sum, st.z2.a, st.z2.cls);
            telemetry.addData("Z3 ST", "has=%s col=%s sum=%.4f a=%.4f cls=%s",
                    st.z3.hasBall, st.z3.color, st.z3.sum, st.z3.a, st.z3.cls);

            telemetry.update();
            sleep(20);
        }
    }

    private static void putZone(TelemetryPacket p, String prefix, ColourZoneDetection.ZoneState z) {
        p.put(prefix + ".hasBall", z.hasBall);
        p.put(prefix + ".color", z.color.toString());
        p.put(prefix + ".color_int", ColourZoneDetection.colorToInt(z.color));
        p.put(prefix + ".cls", z.cls);

        p.put(prefix + ".r", z.r);
        p.put(prefix + ".g", z.g);
        p.put(prefix + ".b", z.b);
        p.put(prefix + ".a", z.a);
        p.put(prefix + ".sum", z.sum);
        p.put(prefix + ".strength", z.strength);
    }

    private static void putSensor(TelemetryPacket p, String prefix, ColourZoneDetection.SensorState s) {
        p.put(prefix + ".cls", s.cls);
        p.put(prefix + ".present", s.present);
        p.put(prefix + ".color", s.color.toString());
        p.put(prefix + ".color_int", ColourZoneDetection.colorToInt(s.color));

        p.put(prefix + ".r", s.r);
        p.put(prefix + ".g", s.g);
        p.put(prefix + ".b", s.b);
        p.put(prefix + ".a", s.a);
        p.put(prefix + ".sum", s.sum);
        p.put(prefix + ".strength", s.strength);
    }

    private static String formatKickOrder(List<ShotOrderPlanner.PlannedShot> plan) {
        if (plan == null || plan.isEmpty()) return "(empty)";
        return plan.stream().map(s -> s.getZone().toString()).collect(Collectors.joining(", "));
    }

    private static String formatKickTimeline(List<ShotOrderPlanner.PlannedShot> plan, double kickUpTimeS, double resetTimeS) {
        if (plan == null || plan.isEmpty()) return "(empty)";
        StringBuilder sb = new StringBuilder();
        double t = 0.0;
        for (int i = 0; i < plan.size(); i++) {
            sb.append(String.format(Locale.US, "t=%.2f %s", t, plan.get(i).getZone()));
            if (i < plan.size() - 1) sb.append(" | ");
            t += (kickUpTimeS + resetTimeS);
        }
        return sb.toString();
    }
}
