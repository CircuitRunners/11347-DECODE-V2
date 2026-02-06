package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.teleOp.competition.MainTeleOp.FORCE_SHOOT_ALL_ZONES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;

import java.util.List;

/**
 * Dashboard-only Auto Shoot Sequence tester:
 * - Detects zone colors
 * - Plans shot order from cipher (PPG/PGP/GPP)
 * - When START_SEQUENCE is toggled true in Dashboard, it:
 *    1) spins shooter to SHOOTER_TARGET_RPM
 *    2) waits until RPM gate passes (or timeout)
 *    3) runs timed kick sequence for each planned zone:
 *          KICK_UP_TIME -> RESET_DOWN_TIME
 */
@Disabled
@Config
@TeleOp(name = "AutoSort Sequence Tester (Dash)", group = "Test")
public class AutoSortSequenceTester extends LinearOpMode {

    // ===================== SRSHub names =====================
    public static String HUB_NAME_A = "srsHubIndexer";
    public static String HUB_NAME_B = "srsHubPlate";

    // ===================== Planner =====================
    public static ShotOrderPlanner.Cipher CIPHER = ShotOrderPlanner.Cipher.PPG;
    public static boolean USE_STABLE_SNAPSHOT = true;

    // ===================== Shooter =====================
    public static double SHOOTER_TARGET_RPM = 4500.0;
    public static double SHOOTER_READY_TOL_RPM = 200.0;
    public static double SHOOTER_READY_TIMEOUT_S = 2.0;

    // ===================== Sequence control (Dashboard) =====================
    public static boolean START_SEQUENCE = false;           // toggle true to start
    public static boolean ABORT_SEQUENCE = false;           // set true to abort immediately

    // ===================== Timing =====================
    public static double KICK_UP_TIME_S = 0.3;
    public static double RESET_DOWN_TIME_S = 0.12;
    public static double BETWEEN_SHOTS_PAUSE_S = 0.15;      // pause between shots -- 0 seems to work,
                                                            // but 0.05 seems to work a bit better

    // ===================== State =====================
    private enum RunState { IDLE, SPINUP_WAIT, KICK_UP, RESET_DOWN, PAUSE, DONE, ABORTED }
    private RunState state = RunState.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime shooterReadyTimer = new ElapsedTime();

    private List<ShotOrderPlanner.PlannedShot> plan = java.util.Collections.emptyList();
    private int stepIdx = 0;
    private boolean lastStart = false;

    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.setTelemetryTransmissionInterval(25);

        ColourZoneDetection czd = new ColourZoneDetection(hardwareMap, HUB_NAME_A, HUB_NAME_B);
        ShotOrderPlanner planner = new ShotOrderPlanner();

        StaticShooter shooter = new StaticShooter(hardwareMap, telemetry);
        Kickers kickers = new Kickers(hardwareMap);

        // Ensure kickers start down
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        waitForStart();

        while (opModeIsActive()) {
            // Update sensors every loop
            czd.update();
            shooter.update();
            ColourZoneDetection.Snapshot snap = USE_STABLE_SNAPSHOT ? czd.getStableSnapshot() : czd.getRawSnapshot();

            // Handle ABORT
            if (ABORT_SEQUENCE) {
                ABORT_SEQUENCE = false;
                abortNow(shooter, kickers);
            }

            // Edge-detect START
            boolean startEdge = START_SEQUENCE && !lastStart;
            lastStart = START_SEQUENCE;

            // WHen START is toggled, start a new run from current snapshot
            if (startEdge && (state == RunState.IDLE || state == RunState.DONE || state == RunState.ABORTED)) {
                START_SEQUENCE = false; // auto reset so you can click it again -- doesnt work rn
                beginRun(planner, snap, shooter, kickers);
            }

            // Run state machine
            runStateMachine(shooter, kickers);

            // Zone colors + plan + state
            TelemetryPacket p = new TelemetryPacket();
            p.put("State", state.toString());
            p.put("Cipher", CIPHER.toString());

            p.put("Z1", snap.z1.color.toString());
            p.put("Z2", snap.z2.color.toString());
            p.put("Z3", snap.z3.color.toString());

            p.put("Planned Order", formatKickOrder(plan));
            p.put("Step", stepIdx + "/" + plan.size());

            double rpm = shooter.getShooterVelocity();
            p.put("Shooter.target_rpm", SHOOTER_TARGET_RPM);
            p.put("Shooter.rpm", rpm);
            p.put("Shooter.ready", isShooterReady(rpm) ? 1 : 0); // graphable

            dash.sendTelemetryPacket(p);

            telemetry.addData("State", state);
            telemetry.addData("Cipher", CIPHER);
            telemetry.addData("Plan", formatKickOrder(plan));
            telemetry.addData("Z1/Z2/Z3", "%s, %s, %s", snap.z1.color, snap.z2.color, snap.z3.color);
            telemetry.addData("Shooter", "target=%.0f rpm=%.0f ready=%s", SHOOTER_TARGET_RPM, rpm, isShooterReady(rpm));
            telemetry.update();

            sleep(20);
        }
    }

    // ===================== Run control =====================

    private void beginRun(ShotOrderPlanner planner,
                          ColourZoneDetection.Snapshot snap,
                          StaticShooter shooter,
                          Kickers kickers) {

        // Build a plan from the CURRENT snapshot
        plan = planner.plan(CIPHER, snap, FORCE_SHOOT_ALL_ZONES);
        stepIdx = 0;

        // Reset kickers down
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        // Spin shooter
        shooter.setTargetRPM(SHOOTER_TARGET_RPM);

        shooterReadyTimer.reset();
        state = RunState.SPINUP_WAIT;
    }

    private void abortNow(StaticShooter shooter, Kickers kickers) {
        // Put everything safe
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        shooter.setTargetRPM(0.0);
        shooter.update();
        shooter.eStop();

        state = RunState.ABORTED;
        timer.reset();
    }

    private void runStateMachine(StaticShooter shooter, Kickers kickers) {
        // Keep shooter commanded whenever we're not idle/done/aborted
        if (state != RunState.IDLE && state != RunState.DONE && state != RunState.ABORTED) {
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
            shooter.update();
        }

        switch (state) {

            case IDLE:
            case DONE:
            case ABORTED:
                return;

            case SPINUP_WAIT: {
                if (plan == null || plan.isEmpty()) {
                    // Nothing to shoot
                    shooter.setTargetRPM(0.0);
                    shooter.update();
                    state = RunState.DONE;
                    return;
                }

                double rpm = shooter.getShooterVelocity();
                boolean ready = isShooterReady(rpm);
                boolean timedOut = shooterReadyTimer.seconds() >= SHOOTER_READY_TIMEOUT_S;

                if (ready || timedOut) {
                    // Start first kick
                    timer.reset();
                    state = RunState.KICK_UP;
                }
                return;
            }

            case KICK_UP: {
                if (stepIdx >= plan.size()) {
                    finishRun(shooter, kickers);
                    return;
                }

                // Kick the planned zone
                Kickers.KickerState unused = Kickers.KickerState.UP;
                ColourZoneDetection.ZoneId zone = plan.get(stepIdx).getZone();

                kickZone(kickers, zone);

                if (timer.seconds() >= KICK_UP_TIME_S) {
                    // Reset all down
                    kickers.resetZoneOne();
                    kickers.resetZoneTwo();
                    kickers.resetZoneThree();

                    timer.reset();
                    state = RunState.RESET_DOWN;
                }
                return;
            }

            case RESET_DOWN: {
                if (timer.seconds() >= RESET_DOWN_TIME_S) {
                    stepIdx++;

                    if (BETWEEN_SHOTS_PAUSE_S > 1e-6) {
                        timer.reset();
                        state = RunState.PAUSE;
                    } else {
                        timer.reset();
                        state = RunState.KICK_UP;
                    }
                }
                return;
            }

            case PAUSE: {
                if (timer.seconds() >= BETWEEN_SHOTS_PAUSE_S) {
                    timer.reset();
                    state = RunState.KICK_UP;
                }
                return;
            }
        }
    }

    private void finishRun(StaticShooter shooter, Kickers kickers) {
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        shooter.setTargetRPM(0.0);
        shooter.update();

        state = RunState.DONE;
        timer.reset();
    }

    // ===================== Helpers =====================

    private boolean isShooterReady(double rpm) {
        return rpm >= (SHOOTER_TARGET_RPM - SHOOTER_READY_TOL_RPM) && rpm > 0.0;
    }

    private static void kickZone(Kickers k, ColourZoneDetection.ZoneId zone) {
        switch (zone) {
            case Z1: k.kickZoneOne(); break;
            case Z2: k.kickZoneTwo(); break;
            case Z3: k.kickZoneThree(); break;
        }
    }

    private static String formatKickOrder(List<ShotOrderPlanner.PlannedShot> plan) {
        if (plan == null || plan.isEmpty()) return "(empty)";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < plan.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(plan.get(i).getZone().toString());
        }
        return sb.toString();
    }
}
