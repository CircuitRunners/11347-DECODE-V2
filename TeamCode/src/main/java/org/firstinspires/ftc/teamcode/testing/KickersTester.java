package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;

@Disabled
@Config
@TeleOp(name = "Kickers Tester (Dashboard)", group = "Test")
public class KickersTester extends LinearOpMode {

    // --- Dashboard controls (dropdowns) ---
    public static Kickers.KickerState Z1_STATE = Kickers.KickerState.DOWN;
    public static Kickers.KickerState Z2_STATE = Kickers.KickerState.DOWN;
    public static Kickers.KickerState Z3_STATE = Kickers.KickerState.DOWN;

    // --- Optional raw overrides (sliders) ---
    public static boolean USE_RAW = false;
    public static double Z1_RAW = 0.50;
    public static double Z2_RAW = 0.50;
    public static double Z3_RAW = 0.50;

    // deadband so small dashboard float jitter doesn't spam writes
    public static double RAW_EPS = 0.002;

    private boolean lastUseRaw = false;
    private Kickers.KickerState lastZ1State = Z1_STATE;
    private Kickers.KickerState lastZ2State = Z2_STATE;
    private Kickers.KickerState lastZ3State = Z3_STATE;

    private double lastZ1Raw = Z1_RAW;
    private double lastZ2Raw = Z2_RAW;
    private double lastZ3Raw = Z3_RAW;

    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.setTelemetryTransmissionInterval(25);

        Kickers kickers = new Kickers(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (USE_RAW) {
                // Raw mode: apply per-zone position sliders
                if (!lastUseRaw
                        || changed(lastZ1Raw, Z1_RAW) || changed(lastZ2Raw, Z2_RAW) || changed(lastZ3Raw, Z3_RAW)) {

                    lastUseRaw = true;

                    // clamp to [0,1] for safety
                    double z1 = clamp01(Z1_RAW);
                    double z2 = clamp01(Z2_RAW);
                    double z3 = clamp01(Z3_RAW);

                    kickers.setZoneOnePos(z1);
                    kickers.setZoneTwoPos(z2);
                    kickers.setZoneThreePos(z3);

                    lastZ1Raw = z1;
                    lastZ2Raw = z2;
                    lastZ3Raw = z3;
                }
            } else {
                // State mode: apply dropdown UP/DOWN per zone
                if (lastUseRaw
                        || lastZ1State != Z1_STATE || lastZ2State != Z2_STATE || lastZ3State != Z3_STATE) {

                    lastUseRaw = false;

                    applyZoneState(kickers);

                    lastZ1State = Z1_STATE;
                    lastZ2State = Z2_STATE;
                    lastZ3State = Z3_STATE;
                }
            }

            TelemetryPacket p = new TelemetryPacket();
            p.put("mode", USE_RAW ? "RAW" : "STATE");

            p.put("z1.state", Z1_STATE.toString());
            p.put("z2.state", Z2_STATE.toString());
            p.put("z3.state", Z3_STATE.toString());

            p.put("z1.raw_cmd", clamp01(Z1_RAW));
            p.put("z2.raw_cmd", clamp01(Z2_RAW));
            p.put("z3.raw_cmd", clamp01(Z3_RAW));

            // what we last actually wrote
            p.put("z1.raw_applied", lastZ1Raw);
            p.put("z2.raw_applied", lastZ2Raw);
            p.put("z3.raw_applied", lastZ3Raw);

            dash.sendTelemetryPacket(p);

            telemetry.addData("Mode", USE_RAW ? "RAW" : "STATE");
            telemetry.addData("Z1", USE_RAW ? "pos=%.3f" : "state=%s", USE_RAW ? clamp01(Z1_RAW) : Z1_STATE.toString());
            telemetry.addData("Z2", USE_RAW ? "pos=%.3f" : "state=%s", USE_RAW ? clamp01(Z2_RAW) : Z2_STATE.toString());
            telemetry.addData("Z3", USE_RAW ? "pos=%.3f" : "state=%s", USE_RAW ? clamp01(Z3_RAW) : Z3_STATE.toString());
            telemetry.update();

            sleep(20);
        }
    }

    private static void applyZoneState(Kickers k) {
        if (Z1_STATE == Kickers.KickerState.UP) k.kickZoneOne();
        else k.resetZoneOne();

        if (Z2_STATE == Kickers.KickerState.UP) k.kickZoneTwo();
        else k.resetZoneTwo();

        if (Z3_STATE == Kickers.KickerState.UP) k.kickZoneThree();
        else k.resetZoneThree();
    }

    private boolean changed(double a, double b) {
        return Math.abs(a - b) > RAW_EPS;
    }

    private static double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}
