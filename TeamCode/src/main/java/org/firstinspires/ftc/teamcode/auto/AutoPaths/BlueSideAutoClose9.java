package org.firstinspires.ftc.teamcode.auto.AutoPaths;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.CypherAprilTagTracker;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ServoTurretTracker;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;
import org.firstinspires.ftc.teamcode.subsystems.vision.WebcamAprilTag;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;
import org.firstinspires.ftc.teamcode.support.BeamBreakHelper;

import java.util.Collections;
import java.util.List;

@Config
@Configurable
@Autonomous(name = "Blue Side Auto Close 6", group = "Blue Autos", preselectTeleOp = "MainTeleOp")
public class BlueSideAutoClose9 extends OpMode {
    // ===================== GOAL / AUTO AIM =====================
    public static Pose TURRET_TARGET_POSE = new Pose(-8, 140);   // field inches
    public static double TURRET_TRIM_DEG = 0.0;                   // optional trim

    // ===================== AUTO-SORT / INDEXING =====================
    public static ShotOrderPlanner.Cipher CIPHER = ShotOrderPlanner.Cipher.PPG;
    public static boolean FORCE_SHOOT_ALL_ZONES = true;
    private boolean cipherLocked = false;
    private ColourZoneDetection.Snapshot initSnap = null;
    private static double INIT_SNAPSHOT_HZ = 20.0; // telemetry refresh rate cap
    private final ElapsedTime initSnapTimer = new ElapsedTime();
    private final ElapsedTime czdStableTimer = new ElapsedTime();
    private static double CZD_STABLE_TIME_S = 0.25;
    private ColourZoneDetection.Snapshot lastStableSnap = null;

    // Shooter ready gate
    public static double SHOOTER_READY_TOL_RPM = 200.0;
    public static double SHOOTER_READY_TIMEOUT_S = 0.8;

    // Kicker timings
    public static double KICK_UP_TIME_S = 0.30;
    public static double RESET_DOWN_TIME_S = 0.12;
    public static double BETWEEN_SHOTS_PAUSE_S = 0.15;

    private int cypherId = -1;
    private BeamBreakHelper outtakeBeamBreak;
    private Thread outtakeThread;

    // Ball counting for each shooting window
    private int beamCountAtRunStart = 0;
    private int plannedShotsThisRun = 0;

    // ===================== AUTO RPM CONTROL (simple distance->rpm table) =====================
    public static double RPM_D0_IN = 24,  RPM_P0 = 3000;
    public static double RPM_D1_IN = 48,  RPM_P1 = 3200;
    public static double RPM_D2_IN = 72,  RPM_P2 = 3400;
    public static double RPM_D3_IN = 96,  RPM_P3 = 3600;
    public static double RPM_D4_IN = 120, RPM_P4 = 3800;

    public static double RPM_MIN = 0.0;
    public static double RPM_MAX = 2850.0; //TODO: reset to 2900

    // ===================== HARDWARE =====================
    private StaticShooter shooter;
    private Kickers kickers;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private ServoTurretTracker turret;

    // ===================== SOFTWARE =====================
    private ShotOrderPlanner planner;
    private ColourZoneDetection czd;

    // ===================== SHOT STATE MACHINE =====================
    private enum RunState { IDLE, SPINUP_WAIT, KICK_UP, RESET_DOWN, PAUSE, DONE, ABORTED }
    private RunState state = RunState.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime shooterReadyTimer = new ElapsedTime();
    private List<ShotOrderPlanner.PlannedShot> plan = Collections.emptyList();
    private ColourZoneDetection.Snapshot snap = null;
    private int stepIdx = 0;

    // The actual commanded RPM (used for readiness gating)
    private double shooterCmdRpm = 0.0;

    // ===================== PATH FOLLOWING =====================
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    // Mirrored positions (x' = 144 - x), but KEEP headings like red (start heading = 0)
    private final Pose startPose = new Pose(33.5, 135.500, Math.toRadians(0));
    private PathChain line1, line2, line3, line4, line5, line6, line7, line8, line67;

    // ===================== BUILD PATHS (MANUALLY MIRRORED POSITIONS, ORIGINAL HEADINGS) =====================
    private void buildPaths() {
        line1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.500, 135.500), new Pose(62.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();

        line67 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(62.00, 100.00), new Pose(44.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                .build();

        line2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 100.000), new Pose(47.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(47.000, 84.000), new Pose(17.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17.000, 84.000), new Pose(44.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 100.000), new Pose(47.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(47.000, 60.000), new Pose(9.500, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.500, 60.000),
                                new Pose(46.000, 64.000), //80
                                new Pose(44.000, 100.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 100.000), new Pose(30.000, 75.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.BLUE.getTagId());

        // Start webcam + pipeline

        // Tracker remembers last seen 21/22/23 and writes AlliancePresets.currentCypher
       

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        intake = new IntakeSubsystem(hardwareMap, telemetry);

        outtake = new OuttakeSubsystem(hardwareMap);
        outtake.setAutomatic(true);
        outtakeBeamBreak = new BeamBreakHelper(hardwareMap, "outtakeBeamBreak", 0);

        kickers = new Kickers(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        czd = new ColourZoneDetection(hardwareMap,
                "z1CSa", "z2CSa", "z3CSa",
                "z1CSb", "z2CSb", "z3CSb");
        planner = new ShotOrderPlanner();
        turret = new ServoTurretTracker(hardwareMap, "turret");
        turret.setEnabled(true);

        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        follower.update();

        buildPaths();
        pathTimer = new Timer();
        pathTimer.resetTimer();
        setPathState(0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Auto:", "Red Side Auto Close 9");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Keep SRS/zone detection "warm" during init
        if (czd != null) {
            czd.update();
            initSnap = czd.getStableSnapshot();
        }

        // Cap telemetry spam a bit
        if (initSnapTimer.seconds() < (1.0 / INIT_SNAPSHOT_HZ)) return;
        initSnapTimer.reset();

        telemetry.addLine("=== INIT PREVIEW ===");
        telemetry.addData("Cypher Tag", cypherId);
        telemetry.addData("Cipher", CIPHER);

        telemetry.addLine("=== ZONE COLORS (Stable Snapshot) ===");
        if (initSnap == null) {
            telemetry.addLine("Snapshot: NULL (not stable / not ready)");
        } else {
            // If Snapshot has a nice toString, this alone might be enough:
            telemetry.addData("Snapshot", initSnap.toString());

            // If Snapshot exposes per-zone info, prefer explicit fields:
            // (Uncomment and adjust names to match your Snapshot class)
            telemetry.addData("Z1", "RAW:%s has=%s", initSnap.z1.color, initSnap.z1.hasBall);
            telemetry.addData("Z2", "RAW:%s has=%s", initSnap.z2.color, initSnap.z2.hasBall);
            telemetry.addData("Z3", "RAW:%s has=%s", initSnap.z3.color, initSnap.z3.hasBall);
        }

        telemetry.update();
    }

    @Override
    public void start() {
        outtakeThread = new Thread(() -> {
            while (!Thread.interrupted()) {
                outtakeBeamBreak.update();
                try {
                    Thread.sleep(1);
                } catch (Exception e) {
                    break;
                }
            }
        });

        outtakeThread.start();

        pathTimer.resetTimer();
        setPathState(0);
        // Ensure global is set even if the last init_loop was missed
    }

    @Override
    public void loop() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }

        telemetry.addLine("=== INIT PREVIEW ===");
        telemetry.addData("Cypher Tag", cypherId);
        telemetry.addData("Cipher", CIPHER);
        telemetry.addData("Tag id", cypherId);

        follower.update();
        czd.update();
        shooter.update();
        outtake.update();

        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeadingRad = robotPose.getHeading();

        double goalX = TURRET_TARGET_POSE.getX();
        double goalY = TURRET_TARGET_POSE.getY();

        Pose2D turretPose = new Pose2D(DistanceUnit.INCH, robotX, robotY, AngleUnit.RADIANS, robotHeadingRad);
        turret.setTargetFieldPointInches(goalX, goalY);
        ServoTurretTracker.TURRET_TRIM_DEG = TURRET_TRIM_DEG;
        turret.setEnabled(true);
        turret.update(turretPose);

        shooter.setTargetRPM(RPM_MAX);

        outtake.updateAutoHoodFromField(robotX, robotY, goalX, goalY);

        boolean shootingActive = (state != RunState.IDLE && state != RunState.DONE && state != RunState.ABORTED);
        if (shootingActive) follower.pausePathFollowing();
        else follower.resumePathFollowing();

        autonomousPathUpdate();

        telemetry.addLine("---- BLUE Side Auto Close 9 ----");
        telemetry.addData("Follower busy?", follower.isBusy());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Shot State", state);
        telemetry.addData("Shooter RPM (meas)", shooter.getShooterVelocity());
        telemetry.addData("Shooter RPM (cmd)", shooterCmdRpm);
        telemetry.addData("Pose", "x=%.2f y=%.2f h=%.1f",
                robotX, robotY, Math.toDegrees(robotHeadingRad));
        telemetry.addData("Beam total", outtakeBeamBreak != null ? outtakeBeamBreak.getBallCount() : -1);
        telemetry.addData("Beam run shots", ballsShotThisRun());
        telemetry.addData("Planned shots", plannedShotsThisRun);
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();

        if (outtakeThread != null) {
            outtakeThread.interrupt();
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();

        if (pathState != 1 && pathState != 7) {
            state = RunState.IDLE;
            plannedShotsThisRun = 0;
            beamCountAtRunStart = (outtakeBeamBreak != null) ? outtakeBeamBreak.getBallCount() : 0;
        }
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(line1);
                    setPathState(-1);
                }
                break;

            case -1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(line67);
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (shooter.isAtTargetThreshold()) {
                        BeginShotSequenceIfIdle();
                        runStateMachine(shooter, kickers);
                    }

                    if (beamRunComplete() || state == RunState.DONE || pathTimer.getElapsedTimeSeconds() > 6.0) {
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intake();
                    follower.followPath(line2);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(line3);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(line4);
                    stopIntake();
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (shooter.isAtTargetThreshold()) {
                        BeginShotSequenceIfIdle();
                        runStateMachine(shooter, kickers);
                    }

                    if (beamRunComplete() || state == RunState.DONE || pathTimer.getElapsedTimeSeconds() > 6.0) {
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intake();
                    follower.followPath(line5);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(line6);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(line7);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    if (shooter.isAtTargetThreshold()) {
                        BeginShotSequenceIfIdle();
                        runStateMachine(shooter, kickers);
                    }

                    if (beamRunComplete() || state == RunState.DONE || pathTimer.getElapsedTimeSeconds() > 6.0) {
                        setPathState(99);
                    }
                }
                break;

            case 99:
                if (!follower.isBusy()) {
                    follower.followPath(line8,false);
                    setPathState(100);
                }
                return;

            case 100:
                if (!follower.isBusy()) {
                    AlliancePresets.setCurrentPose2D(new Pose2D(
                            DistanceUnit.INCH,
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            AngleUnit.RADIANS,
                            follower.getHeading()
                    ));
                    AlliancePresets.setCurrentPose(follower.getPose());
                    RPM_MAX = 0;
                    shooter.eStop();
                    follower.breakFollowing();
                }
                break;
        }
    }

    private void BeginShotSequenceIfIdle() {
        if (state != RunState.IDLE && state != RunState.DONE && state != RunState.ABORTED) return;

        ColourZoneDetection.Snapshot s = czd.getRawSnapshot();
        if (s == null) return;

        snap = s;

        telemetry.addData("Z1", "RAW:%s has=%s", s.z1.color, s.z1.hasBall);
        telemetry.addData("Z2", "RAW:%s has=%s", s.z2.color, s.z2.hasBall);
        telemetry.addData("Z3", "RAW:%s has=%s", s.z3.color, s.z3.hasBall);
        beginRun(planner, snap, shooter, kickers);
    }

    private void beginRun(ShotOrderPlanner planner,
                          ColourZoneDetection.Snapshot snap,
                          StaticShooter shooter,
                          Kickers kickers) {
        boolean force = FORCE_SHOOT_ALL_ZONES;

        if (force && snap != null) {
            boolean anyPresent =
                    (snap.z1.hasBall && snap.z1.color != ColourZoneDetection.BallColor.NONE) ||
                            (snap.z2.hasBall && snap.z2.color != ColourZoneDetection.BallColor.NONE) ||
                            (snap.z3.hasBall && snap.z3.color != ColourZoneDetection.BallColor.NONE);
            force = !anyPresent;
        }

        plan = planner.plan(CIPHER, snap, force);

        stepIdx = 0;
        plannedShotsThisRun = (plan == null) ? 0 : plan.size();

        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        shooterReadyTimer.reset();
        timer.reset();
        state = RunState.SPINUP_WAIT;
    }

    private void runStateMachine(StaticShooter shooter, Kickers kickers) {
        switch (state) {
            case SPINUP_WAIT:
                if (plan == null || plan.isEmpty()) { state = RunState.DONE; return; }
                if (isShooterReady(shooter.getShooterVelocity()) ||
                        shooterReadyTimer.seconds() >= SHOOTER_READY_TIMEOUT_S) {
                    timer.reset();
                    state = RunState.KICK_UP;
                }
                break;

            case KICK_UP:
                if (stepIdx >= plan.size()) { finishRun(kickers); return; }
                kickZone(kickers, plan.get(stepIdx).getZone());
                if (timer.seconds() >= KICK_UP_TIME_S) {
                    kickers.resetZoneOne();
                    kickers.resetZoneTwo();
                    kickers.resetZoneThree();
                    timer.reset();
                    state = RunState.RESET_DOWN;
                }
                break;

            case RESET_DOWN:
                if (timer.seconds() >= RESET_DOWN_TIME_S) {
                    stepIdx++;
                    state = (BETWEEN_SHOTS_PAUSE_S > 1e-6) ? RunState.PAUSE : RunState.KICK_UP;
                    timer.reset();
                }
                break;

            case PAUSE:
                if (timer.seconds() >= BETWEEN_SHOTS_PAUSE_S) {
                    timer.reset();
                    state = RunState.KICK_UP;
                }
                break;

            default:
                break;
        }
    }

    private void finishRun(Kickers kickers) {
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();
        state = RunState.DONE;
    }

    private boolean isShooterReady(double measuredRpm) {
        return shooterCmdRpm > 0.0 && measuredRpm >= (shooterCmdRpm - SHOOTER_READY_TOL_RPM);
    }

    private static void kickZone(Kickers k, ColourZoneDetection.ZoneId zone) {
        switch (zone) {
            case Z1: k.kickZoneOne(); break;
            case Z2: k.kickZoneTwo(); break;
            case Z3: k.kickZoneThree(); break;
        }
    }

    private double interpRpm(double dist) {
        if (dist <= RPM_D0_IN) return RPM_P0;
        if (dist >= RPM_D4_IN) return RPM_P4;

        if (dist <= RPM_D1_IN) return lerp(RPM_D0_IN, RPM_P0, RPM_D1_IN, RPM_P1, dist);
        if (dist <= RPM_D2_IN) return lerp(RPM_D1_IN, RPM_P1, RPM_D2_IN, RPM_P2, dist);
        if (dist <= RPM_D3_IN) return lerp(RPM_D2_IN, RPM_P2, RPM_D3_IN, RPM_P3, dist);
        return lerp(RPM_D3_IN, RPM_P3, RPM_D4_IN, RPM_P4, dist);
    }

    private static double lerp(double x0, double y0, double x1, double y1, double x) {
        double t = (x - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private int ballsShotThisRun() {
        if (outtakeBeamBreak == null) return 0;
        return outtakeBeamBreak.getBallCount() - beamCountAtRunStart;
    }

    private boolean beamRunComplete() {
        return plannedShotsThisRun > 0 && ballsShotThisRun() >= plannedShotsThisRun;
    }

    private void intake() {
        intake.intakeChub(1);
    }

    private void stopIntake() {
        intake.stop();
    }
}