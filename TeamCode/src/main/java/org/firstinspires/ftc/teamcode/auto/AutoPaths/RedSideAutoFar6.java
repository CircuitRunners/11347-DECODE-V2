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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.OpenCVPipelines.PurpleGreenBlobPipeline;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ServoTurretTracker;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;
import org.firstinspires.ftc.teamcode.support.BeamBreakHelper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Collections;
import java.util.List;

@Config
@Configurable
@Autonomous(name = "Red Side Auto Far 6 (Blob Chase Drop-in)", group = "Red Autos", preselectTeleOp = "MainTeleOp")
public class RedSideAutoFar6 extends OpMode {

    // ===================== GOAL / AUTO AIM =====================
    public static Pose TURRET_TARGET_POSE = new Pose(140, 136);
    public static double TURRET_TRIM_DEG = 0.0;

    // ===================== AUTO-SORT / INDEXING =====================
    public static ShotOrderPlanner.Cipher CIPHER = ShotOrderPlanner.Cipher.PPG;
    public static boolean FORCE_SHOOT_ALL_ZONES = true;

    private ColourZoneDetection.Snapshot initSnap = null;
    private static double INIT_SNAPSHOT_HZ = 20.0;
    private final ElapsedTime initSnapTimer = new ElapsedTime();

    // Shooter ready gate
    public static double SHOOTER_READY_TOL_RPM = 100.0;
    public static double SHOOTER_READY_TIMEOUT_S = 0.8;

    // Kicker timings
    public static double KICK_UP_TIME_S = 0.30;
    public static double RESET_DOWN_TIME_S = 0.12;
    public static double BETWEEN_SHOTS_PAUSE_S = 0.15;

    private BeamBreakHelper outtakeBeamBreak;
    private Thread outtakeThread;

    private int beamCountAtRunStart = 0;
    private int plannedShotsThisRun = 0;

    // ===================== SHOOTER RPM =====================
    public static double RPM_MAX = 4000.0;
    private double shooterCmdRpm = 0.0;

    // ===================== HARDWARE =====================
    private StaticShooter shooter;
    private Kickers kickers;
    private IntakeSubsystem intake;
    private ServoTurretTracker turret;

    // Manual motor drive (for blob chase)
    private MecanumDrivebase drive;

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

    // ===================== PATH FOLLOWING =====================
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    private final Pose startPose = new Pose(104.0, 8.2, Math.toRadians(0));
    private PathChain line1, line2, line3, line4, line5, line6;

    // ===================== BLOB CHASE =====================
    private OpenCvCamera blobCam;
    private PurpleGreenBlobPipeline blobPipeline;

    public static String BLOB_WEBCAM_NAME = "RedWebcam";
    public static double BLOB_CENTER_DEADBAND = 0.12;
    public static double BLOB_K_TURN = 0.25;
    public static double BLOB_MAX_TURN = 0.20;
    public static double BLOB_FWD_BASE = 0.10;
    public static double BLOB_FWD_CENTERED = 0.14;
    public static double BLOB_MAX_FWD = -0.18;
    public static double BLOB_SCAN_TURN = 0.10;
    public static double BLOB_SCAN_FWD = 0.05;

    // NEW: treat "fills camera" as "close enough"
    // If your pipeline's "area" is contour area in pixels, then compare against frame area.
    // Start with 0.35-0.55 and tune.
    public static double BLOB_CLOSE_AREA_FRAC = 0.45;

    public static int BALL_LIMIT = 3;
    public static double X_LIMIT = 130.0;

    public static double BLOB_TURN_SIGN = 1.0;

    // ===================== BUILD PATHS =====================
    private void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(104.000, 8.200),
                        new Pose(109.000, 20.000),
                        new Pose(119.000, 13.500)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0.0))
                .build();

        line2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(119.000, 13.500),
                        new Pose(134.000, 13.500)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
                .build();

        line3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(134.000, 14.000),
                        new Pose(119.000, 9.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
                .build();

        line4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(119.000, 9.000),
                        new Pose(135.000, 9.000)
                ))
                .setTangentHeadingInterpolation()
                .build();

        line5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(135.000, 9.000),
                        new Pose(124.000, 30.000),
                        new Pose(104.000, 8.200)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0))
                .build();

        line6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(104.000, 8.200),
                        new Pose(109.000, 15.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0.0))
                .build();
    }

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.RED.getTagId());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Manual drive for blob chase
        drive = new MecanumDrivebase(hardwareMap, true, true, follower);

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        intake = new IntakeSubsystem(hardwareMap, telemetry);

        outtakeBeamBreak = new BeamBreakHelper(hardwareMap, "outtakeBeamBreak", 0);

        kickers = new Kickers(hardwareMap);

        czd = new ColourZoneDetection(hardwareMap,
                "z1CSa", "z2CSa", "z3CSa",
                "z1CSb", "z2CSb", "z3CSb");
        planner = new ShotOrderPlanner();

        turret = new ServoTurretTracker(hardwareMap, "turret");
        turret.setEnabled(true);

        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        // ===== Start blob webcam + pipeline =====
        blobPipeline = new PurpleGreenBlobPipeline();

        int camMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        blobCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, BLOB_WEBCAM_NAME),
                camMonitorViewId
        );
        blobCam.setPipeline(blobPipeline);

        blobCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                blobCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("BlobCam error", errorCode);
            }
        });

        follower.update();
        buildPaths();

        pathTimer = new Timer();
        pathTimer.resetTimer();
        setPathState(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (czd != null) {
            czd.update();
            initSnap = czd.getStableSnapshot();
        }

        if (initSnapTimer.seconds() < (1.0 / INIT_SNAPSHOT_HZ)) return;
        initSnapTimer.reset();

        telemetry.addLine("=== INIT PREVIEW ===");
        telemetry.addData("Cipher", CIPHER);

        telemetry.addLine("=== ZONE COLORS (Stable Snapshot) ===");
        if (initSnap == null) {
            telemetry.addLine("Snapshot: NULL (not stable / not ready)");
        } else {
            telemetry.addData("Snapshot", initSnap.toString());
            telemetry.addData("Z1", "RAW:%s has=%s", initSnap.z1.color, initSnap.z1.hasBall);
            telemetry.addData("Z2", "RAW:%s has=%s", initSnap.z2.color, initSnap.z2.hasBall);
            telemetry.addData("Z3", "RAW:%s has=%s", initSnap.z3.color, initSnap.z3.hasBall);
        }

        telemetry.addData("BlobTarget", blobPipeline != null && blobPipeline.hasTarget);
        if (blobPipeline != null) telemetry.addData("Blob cxNorm", "%.2f", blobPipeline.cxNorm);

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
    }

    @Override
    public void loop() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }

        follower.update();
        czd.update();

        // ===================== AUTO AIM / SHOOTER RPM =====================
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

        shooterCmdRpm = RPM_MAX;
        shooter.setTargetRPM(shooterCmdRpm);

        // Freeze follower while shooting (manual drive will be forced 0 in those states anyway)
        boolean shootingActive = (state != RunState.IDLE && state != RunState.DONE && state != RunState.ABORTED);
        if (shootingActive) follower.pausePathFollowing();
        else follower.resumePathFollowing();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shot State", state);
        telemetry.addData("Pose", "x=%.2f y=%.2f h=%.1f", robotX, robotY, Math.toDegrees(robotHeadingRad));
        telemetry.addData("Balls(in zones)", ballsInRobotEstimate());
        telemetry.addData("BlobTarget", blobPipeline != null && blobPipeline.hasTarget);
        if (blobPipeline != null) telemetry.addData("cxNorm", "%.2f", blobPipeline.cxNorm);
        telemetry.update();
    }

    @Override
    public void stop() {
        try {
            if (blobCam != null) blobCam.stopStreaming();
        } catch (Exception ignored) {}

        drive.drive(0, 0, 0);
        follower.breakFollowing();
        shooter.eStop();
        stopIntake();
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

    // ===================== MAIN AUTO LOGIC =====================
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    setPathState(1);
                }
                break;

            case 1:
                if (shooter.isAtTargetThreshold()) {
                    BeginShotSequenceIfIdle();
                    runStateMachine(shooter, kickers);
                }

                if (beamRunComplete() || state == RunState.DONE || pathTimer.getElapsedTimeSeconds() > 6.0) {
                    setPathState(2);
                }
                break;

            case 2:
                // Run your normal move, then switch into blob chase (manual motor control)
                if (!follower.isBusy()) {
                    intake();
                    follower.setMaxPower(0.5);
                    follower.followPath(line1);
                    setPathState(3);
                }
                break;

            case 3:
                // Enter blob chase mode when line1 completes
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();
                    drive.drive(0, 0, 0);
                    setPathState(20);
                }
                break;

            case 20: {
                Pose p = follower.getPose();
                double x = p.getX();

                int balls = ballsInRobotEstimate();
                boolean stopBecauseFull = balls >= BALL_LIMIT;
                boolean stopBecauseX = x >= X_LIMIT;

                // DEBUG: show why it stops
                telemetry.addData("CHASE x", "%.1f", x);
                telemetry.addData("CHASE balls", balls);
                telemetry.addData("stopFull", stopBecauseFull);
                telemetry.addData("stopX", stopBecauseX);

                if (stopBecauseFull || stopBecauseX) {
                    stopIntake();
                    drive.drive(0, 0, 0);

                    follower.resumePathFollowing();
                    follower.setMaxPower(0.6);
                    follower.followPath(line5);
                    setPathState(7);
                    break;
                }

                intake();

                if (blobPipeline == null || !blobPipeline.hasTarget) {
                    // robot-centric scan
                    drive.drive(BLOB_SCAN_FWD, 0.0, BLOB_SCAN_TURN);
                    break;
                }

                final double frameArea = 640.0 * 480.0;
                final double areaFrac = (blobPipeline.area <= 0.0) ? 0.0 : (blobPipeline.area / frameArea);
                final boolean closeEnough = areaFrac >= BLOB_CLOSE_AREA_FRAC;

                telemetry.addData("hasTarget", true);
                telemetry.addData("areaFrac", "%.2f", areaFrac);

                if (closeEnough) {
                    // straight robot-centric forward
                    double forward = clamp(BLOB_FWD_CENTERED, BLOB_MAX_FWD, 0.0);
                    drive.drive(forward, 0.0, 0.0);
                    telemetry.addData("Blob mode", "CLOSE -> straight");
                    break;
                }

                double cx = blobPipeline.cxNorm; // [-1..1]
                boolean centered = Math.abs(cx) <= BLOB_CENTER_DEADBAND;

                double turn = BLOB_TURN_SIGN * (BLOB_K_TURN * cx);
                turn = clamp(turn, -BLOB_MAX_TURN, BLOB_MAX_TURN);

                double forward = centered ? BLOB_FWD_CENTERED : BLOB_FWD_BASE;
                forward = clamp(forward, 0.0, BLOB_MAX_FWD);

                forward *= (1.0 - clamp(Math.abs(turn) / BLOB_MAX_TURN, 0.0, 1.0) * 0.6);

                // robot-centric chase
                drive.drive(forward, 0.0, turn);

                telemetry.addData("cxNorm", "%.2f", cx);
                telemetry.addData("cmdFwd", "%.2f", forward);
                telemetry.addData("cmdTurn", "%.2f", turn);
                break;
            }

            case 7:
                // stop manual drive so follower can take over
                drive.drive(0, 0, 0);

                if (!follower.isBusy()) {
                    if (shooter.isAtTargetThreshold()) {
                        BeginShotSequenceIfIdle();
                        runStateMachine(shooter, kickers);
                    }

                    if (beamRunComplete() || state == RunState.DONE || pathTimer.getElapsedTimeSeconds() > 6.0) {
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(line6);
                    setPathState(9);
                }
                break;

            case 9:
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
                    stopIntake();
                    drive.drive(0, 0, 0);
                }
                break;

            default:
                break;
        }
    }

    // ===================== SHOT SEQUENCE HELPERS =====================
    private void BeginShotSequenceIfIdle() {
        if (state != RunState.IDLE && state != RunState.DONE && state != RunState.ABORTED) return;

        ColourZoneDetection.Snapshot s = czd.getRawSnapshot();
        if (s == null) return;

        snap = s;
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
        intake.intakeEhub(1);
    }

    private void stopIntake() {
        intake.stop();
    }

    private int ballsInRobotEstimate() {
        ColourZoneDetection.Snapshot s = czd.getRawSnapshot();
        if (s == null) return 0;

        int count = 0;
        if (s.z1.hasBall) count++;
        if (s.z2.hasBall) count++;
        if (s.z3.hasBall) count++;
        return count;
    }
}