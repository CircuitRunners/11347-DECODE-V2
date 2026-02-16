package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;

@Disabled
@TeleOp(name = "CZ Detect Tester (2 SRSHubs)", group = "Test")
public class ColourZoneDetectionTester extends LinearOpMode {

    // MUST match RC config names
    private static final String HUB_NAME_A = "srsHubIndexer";
    private static final String HUB_NAME_B = "srsHubPlate";

    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.setTelemetryTransmissionInterval(25);

        ColourZoneDetection czd = new ColourZoneDetection(hardwareMap,
                "z1CSa", "z2CSa", "z3CSa",
                "z1CSb", "z2CSb", "z3CSb");

        waitForStart();

        while (opModeIsActive()) {
            czd.update();

            ColourZoneDetection.Snapshot raw = czd.getRawSnapshot();
            ColourZoneDetection.Snapshot st  = czd.getStableSnapshot();

            TelemetryPacket p = new TelemetryPacket();

            // per-zone combined state (raw + stable)
            putZoneCombined(p, "z1", raw.z1, st.z1);
            putZoneCombined(p, "z2", raw.z2, st.z2);
            putZoneCombined(p, "z3", raw.z3, st.z3);

            // each sensor state
            putSensor(p, "z1.a", czd.getZ1A());
            putSensor(p, "z1.b", czd.getZ1B());
            putSensor(p, "z2.a", czd.getZ2A());
            putSensor(p, "z2.b", czd.getZ2B());
            putSensor(p, "z3.a", czd.getZ3A());
            putSensor(p, "z3.b", czd.getZ3B());

            dash.sendTelemetryPacket(p);

            telemetry.addData("Z1", "RAW:%s has=%s | ST:%s has=%s", raw.z1.color, raw.z1.hasBall, st.z1.color, st.z1.hasBall);
            telemetry.addData("Z2", "RAW:%s has=%s | ST:%s has=%s", raw.z2.color, raw.z2.hasBall, st.z2.color, st.z2.hasBall);
            telemetry.addData("Z3", "RAW:%s has=%s | ST:%s has=%s", raw.z3.color, raw.z3.hasBall, st.z3.color, st.z3.hasBall);
            telemetry.update();

            sleep(20);
        }
    }

    private static void putZoneCombined(TelemetryPacket p, String prefix,
                                        ColourZoneDetection.ZoneState raw,
                                        ColourZoneDetection.ZoneState st) {

        p.put(prefix + ".raw.color", raw.color.toString());
        p.put(prefix + ".raw.hasBall", raw.hasBall);
        p.put(prefix + ".st.color", st.color.toString());
        p.put(prefix + ".st.hasBall", st.hasBall);

        p.put(prefix + ".raw.color_int", ColourZoneDetection.colorToInt(raw.color));
        p.put(prefix + ".st.color_int", ColourZoneDetection.colorToInt(st.color));
        p.put(prefix + ".raw.hasBall_int", raw.hasBall ? 1 : 0);
        p.put(prefix + ".st.hasBall_int", st.hasBall ? 1 : 0);

        // chosen sensor raw (graphable)
        p.put(prefix + ".chosen.r", raw.r);
        p.put(prefix + ".chosen.g", raw.g);
        p.put(prefix + ".chosen.b", raw.b);

        p.put(prefix + ".chosen.rgb_sum", raw.r + raw.g + raw.b);
        p.put(prefix + ".chosen.gb_sum", raw.g + raw.b);
        p.put(prefix + ".chosen.g_minus_r", raw.g - raw.r);
        p.put(prefix + ".chosen.g_minus_b", raw.g - raw.b);
    }

    private static void putSensor(TelemetryPacket p, String prefix, ColourZoneDetection.SensorState s) {
        p.put(prefix + ".present", s.present);
        p.put(prefix + ".present_int", s.present ? 1 : 0);
        p.put(prefix + ".color", s.color.toString());
        p.put(prefix + ".color_int", ColourZoneDetection.colorToInt(s.color));

        p.put(prefix + ".strength", s.strength);
        p.put(prefix + ".r", s.r);
        p.put(prefix + ".g", s.g);
        p.put(prefix + ".b", s.b);
        p.put(prefix + ".a", s.a);
        p.put(prefix + ".rgb_sum", s.r + s.g + s.b);
    }
}
