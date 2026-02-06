package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;

/**
 * Dashboard-only combined tester:
 * - Kickers: dropdown UP/DOWN per zone OR raw position sliders per zone
 * - Shooter: set target RPM from Dashboard, start/stop, live RPM + at-target status
 *
 * No controller required.
 */
@Disabled
@Config
@TeleOp(name = "Kickers + Shooter Tester (Dashboard)", group = "Test")
public class KickersShooterTester extends LinearOpMode {

    // ===================== Kickers controls =====================
    public static boolean K_USE_RAW = false;
    public static double K_RAW_EPS = 0.002;

    public static Kickers.KickerState K_Z1_STATE = Kickers.KickerState.DOWN;
    public static Kickers.KickerState K_Z2_STATE = Kickers.KickerState.DOWN;
    public static Kickers.KickerState K_Z3_STATE = Kickers.KickerState.DOWN;

    public static double K_Z1_RAW = 0.50;
    public static double K_Z2_RAW = 0.50;
    public static double K_Z3_RAW = 0.50;

    // ===================== Shooter controls =====================
    public static boolean S_ENABLED = false;
    public static double S_TARGET_RPM = 3000.0;

    // Useful while tuning
    public static boolean S_APPLY_PIDF_EVERY_LOOP = false;

    // ===================== State tracking =====================
    private boolean lastKUseRaw = false;
    private Kickers.KickerState lastZ1State = K_Z1_STATE;
    private Kickers.KickerState lastZ2State = K_Z2_STATE;
    private Kickers.KickerState lastZ3State = K_Z3_STATE;

    private double lastZ1Raw = K_Z1_RAW;
    private double lastZ2Raw = K_Z2_RAW;
    private double lastZ3Raw = K_Z3_RAW;

    private boolean lastShooterEnabled = false;
    private double lastShooterTarget = S_TARGET_RPM;

    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.setTelemetryTransmissionInterval(25);

        Kickers kickers = new Kickers(hardwareMap);
        StaticShooter shooter = new StaticShooter(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            // ===================== Kickers apply =====================
            if (K_USE_RAW) {
                if (!lastKUseRaw
                        || changed(lastZ1Raw, K_Z1_RAW) || changed(lastZ2Raw, K_Z2_RAW) || changed(lastZ3Raw, K_Z3_RAW)) {

                    lastKUseRaw = true;

                    double z1 = clamp01(K_Z1_RAW);
                    double z2 = clamp01(K_Z2_RAW);
                    double z3 = clamp01(K_Z3_RAW);

                    kickers.setZoneOnePos(z1);
                    kickers.setZoneTwoPos(z2);
                    kickers.setZoneThreePos(z3);

                    lastZ1Raw = z1;
                    lastZ2Raw = z2;
                    lastZ3Raw = z3;
                }
            } else {
                if (lastKUseRaw
                        || lastZ1State != K_Z1_STATE || lastZ2State != K_Z2_STATE || lastZ3State != K_Z3_STATE) {

                    lastKUseRaw = false;

                    applyKickerState(kickers);

                    lastZ1State = K_Z1_STATE;
                    lastZ2State = K_Z2_STATE;
                    lastZ3State = K_Z3_STATE;
                }
            }

            // ===================== Shooter apply =====================
            if (!S_ENABLED) {
                if (lastShooterEnabled) {
                    shooter.setTargetRPM(0.0);
                    shooter.update();
                    // shooter.eStop(); // optional hard stop
                    lastShooterEnabled = false;
                }
            } else {
                // enabled
                if (!lastShooterEnabled || Math.abs(S_TARGET_RPM - lastShooterTarget) > 1e-6) {
                    shooter.setTargetRPM(S_TARGET_RPM);
                    lastShooterEnabled = true;
                    lastShooterTarget = S_TARGET_RPM;
                }

                if (S_APPLY_PIDF_EVERY_LOOP) shooter.applyPIDF();
                shooter.update();
            }

            // ===================== Telemetry (Dashboard) =====================
            TelemetryPacket p = new TelemetryPacket();

            // Kickers summary
            p.put("K.mode", K_USE_RAW ? "RAW" : "STATE");
            p.put("K.z1.state", K_Z1_STATE.toString());
            p.put("K.z2.state", K_Z2_STATE.toString());
            p.put("K.z3.state", K_Z3_STATE.toString());
            p.put("K.z1.raw_cmd", clamp01(K_Z1_RAW));
            p.put("K.z2.raw_cmd", clamp01(K_Z2_RAW));
            p.put("K.z3.raw_cmd", clamp01(K_Z3_RAW));
            p.put("K.z1.raw_applied", lastZ1Raw);
            p.put("K.z2.raw_applied", lastZ2Raw);
            p.put("K.z3.raw_applied", lastZ3Raw);

            // Shooter summary
            double rpm = shooter.getShooterVelocity();
            p.put("S.enabled", S_ENABLED);
            p.put("S.target_rpm", S_TARGET_RPM);
            p.put("S.rpm", rpm);
            p.put("S.at_target", shooter.isAtTargetThreshold() ? 1 : 0); // graphable
            p.put("S.active", shooter.isActive() ? 1 : 0);               // graphable
            p.put("S.current_amps", shooter.getMotorVoltage());

            // Graphables
            p.put("S.rpm_err", S_TARGET_RPM - rpm);

            dash.sendTelemetryPacket(p);

            // ===================== Telemetry (Driver Station) =====================
            telemetry.addData("K", "mode=%s z1=%s z2=%s z3=%s",
                    K_USE_RAW ? "RAW" : "STATE", K_Z1_STATE, K_Z2_STATE, K_Z3_STATE);
            if (K_USE_RAW) {
                telemetry.addData("K RAW", "z1=%.3f z2=%.3f z3=%.3f", clamp01(K_Z1_RAW), clamp01(K_Z2_RAW), clamp01(K_Z3_RAW));
            }

            telemetry.addData("S", "enabled=%s target=%.0f rpm=%.0f err=%.0f atTarget=%s",
                    S_ENABLED, S_TARGET_RPM, rpm, (S_TARGET_RPM - rpm), shooter.isAtTargetThreshold());

            telemetry.update();

            sleep(20);
        }
    }

    private static void applyKickerState(Kickers k) {
        if (K_Z1_STATE == Kickers.KickerState.UP) k.kickZoneOne(); else k.resetZoneOne();
        if (K_Z2_STATE == Kickers.KickerState.UP) k.kickZoneTwo(); else k.resetZoneTwo();
        if (K_Z3_STATE == Kickers.KickerState.UP) k.kickZoneThree(); else k.resetZoneThree();
    }

    private boolean changed(double last, double now) {
        return Math.abs(last - now) > K_RAW_EPS;
    }

    private static double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}
