package org.firstinspires.ftc.teamcode.auto.AutoPaths;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.BeamBreakHelper;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ServoTurretTracker;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;
import org.firstinspires.ftc.teamcode.teleOp.competition.MainTeleOp;

import java.util.Collections;
import java.util.List;


@Config
@Configurable
@Autonomous(name="Red Side Auto Far",group="Red Autos", preselectTeleOp="MainTeleOp")
public class RedSideAutoFar9 extends OpMode {
    // ---- Auto-sort / indexing ----
    public static ShotOrderPlanner.Cipher CIPHER = ShotOrderPlanner.Cipher.PPG;
    public static boolean START_SEQUENCE = false;
    public static boolean ABORT_SEQUENCE = false;
    public static boolean FORCE_SHOOT_ALL_ZONES = true;

    public static double SHOOTER_TARGET_RPM = 4000.0;
    public static boolean SHOOTER_ENABLED = true;
    public static double SHOOTER_READY_TOL_RPM = 200.0;
    public static double SHOOTER_READY_TIMEOUT_S = 0.5;

    public static double KICK_UP_TIME_S = 0.30;
    public static double RESET_DOWN_TIME_S = 0.12;
    public static double BETWEEN_SHOTS_PAUSE_S = 0.15;

    public static Pose TURRET_TARGET_POSE = new Pose(144, 144);
    public static double FIELD_SIZE_IN = 144.0;

    private double goalX_odom() {
        return FIELD_SIZE_IN - TURRET_TARGET_POSE.getX();
    }

    private double goalY_odom() {
        return FIELD_SIZE_IN - TURRET_TARGET_POSE.getY();
    }

    // ===================== Hardware =====================
    private StaticShooter shooter;
    private Kickers kickers;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private ServoTurretTracker turret;
    private GoBildaPinpointDriver pinpoint;

    // ===================== Software =====================
    private ShotOrderPlanner planner;
    private ColourZoneDetection czd;

    // ===================== Auto-sort state machine =====================
    private enum RunState { IDLE, SPINUP_WAIT, KICK_UP, RESET_DOWN, PAUSE, DONE, ABORTED }

    private RunState state = RunState.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime shooterReadyTimer = new ElapsedTime();
    private List<ShotOrderPlanner.PlannedShot> plan = Collections.emptyList();
    private ColourZoneDetection.Snapshot snap;
    private int stepIdx = 0;
    private boolean lastStart = false;


    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private int ballsToShoot;
    private int FAR_SHOOTER_POWER = 4000;
    private int LOWEST_POWER = 3800;
    private double SHOOTING_ANGLE = 67;
    private final double hoodAngle = 0.76;
    Timer shootTime = new Timer();

    private boolean intaking, transfering, scoring, moving;

    private boolean headingLockEnabled;
    //private BeamBreakHelper intakeBeamBreak, outtakeBeamBreak;
    //private Thread outtakeThread;
    private final Pose startPose = new Pose(104.0, 8.2, Math.toRadians(0));
    private PathChain line1, line2, line3, line4, line5, line6;

    public void buildPaths() {
        line1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(104.000, 8.200),
                                new Pose(109.000, 20.000),
                                new Pose(119.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(SHOOTING_ANGLE),
                        Math.toRadians(0.0)
                )
                .build();
        line2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(119.000, 14.000),
                                new Pose(135.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(0.0)
                )
                .build();

        line3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 14.000),
                                new Pose(119.000, 9.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(0.0)
                )
                .build();

        line4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(119.000, 9.000),
                                new Pose(135.000, 9.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        line5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135.000, 9.000),
                                new Pose(124.000, 30.000),
                                new Pose(104.000, 8.200)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(0)
                )
                .build();

        line6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(104.000, 8.200),
                                new Pose(109.000, 15.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0.0)
                )
                .build();

    }


    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ballsToShoot = 3;
        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(hardwareMap);
        kickers = new Kickers(hardwareMap);
        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.RED.getTagId());

        outtake.setAutomatic(false);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        czd = new ColourZoneDetection(hardwareMap, "srsHubIndexer", "srsHubPlate");
        planner = new ShotOrderPlanner();

        turret = new ServoTurretTracker(hardwareMap, "turret");
        turret.setEnabled(false);
//        turret.setTargetFieldPointInches(TURRET_TARGET_POSE.getX(), TURRET_TARGET_POSE.getY());

        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        follower.update();

        buildPaths();
        pathTimer = new Timer();
        pathTimer.resetTimer();

        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.RED.getTagId());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Team ID:", AlliancePresets.getAllianceShooterTag());
        telemetry.addData("Pinpoint X", follower.getPose().getX());
        telemetry.addData("Pinpoint Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
    }

    @Override
    public void start() {
//        outtakeThread = new Thread(() -> {
//            while (!Thread.interrupted()) {
//                outtakeBeamBreak.update();
//                try {
//                    Thread.sleep(1);
//                } catch (Exception e) {
//                    break;
//                }
//            }
//        });

        //outtakeThread.start();
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        //intakeBeamBreak.update();
        follower.update();
        czd.update();
        shooter.update();
        outtake.update();

        Pose currentPose = follower.getPose();
        double gx = currentPose.getX();
        double gy = currentPose.getY();

        Pose2D turretPose = new Pose2D(DistanceUnit.INCH, gx, gy, AngleUnit.RADIANS, currentPose.getHeading() - (Math.toRadians(90)));

        turret.setTargetFieldPointInches(gx, gy);
        ServoTurretTracker.TURRET_TRIM_DEG = 0.0;
        turret.setEnabled(true);

        turret.update(turretPose);

        if (state == RunState.IDLE || state == RunState.DONE || state == RunState.ABORTED) {
            follower.resumePathFollowing();
        }

        //limelight.update();
        autonomousPathUpdate();

        telemetry.addLine("----  RED Side Auto Far 9 (Loading Zone)  ----");
        telemetry.addLine();
        telemetry.addData("Follower busy?", follower.isBusy());
        telemetry.addData("Path State: ", pathState);
        telemetry.addData("Shooter Velo: ", shooter.getShooterVelocity());
        //telemetry.addData("Balls Shot", outtakeBeamBreak.getBallCount());
        //telemetry.addData("Ball Held", intakeBeamBreak.getBallCount() % 3);
        telemetry.addData("Balls to Shoot", ballsToShoot);
        //telemetry.addData("Outtake Raw", outtakeBeamBreak.isBeamBroken());
        telemetry.addData("Timer: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.1f", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
//        if (outtakeThread != null) {
//            outtakeThread.interrupt();
//        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    shooter.setTargetRPM(FAR_SHOOTER_POWER);
                    outtake.aimScoring(hoodAngle);
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 6) {
                            BeginShotSequence();
                            snap = czd.getStableSnapshot();
                            runStateMachine(shooter, kickers);
                    } else {
                        intake();
                        setPathState(2);
                    }
                }

                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(line1);
                    shootTime.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(line2);
                    setPathState(-3);
                }
                break;
            case -3:
                if (!follower.isBusy()) {
                    follower.followPath(line3);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    ballsToShoot = 3;
                    follower.followPath(line4);
                    setPathState(5);
                }
                break;


            case 5:
                if (!follower.isBusy()) {
                    shooter.setTargetRPM(FAR_SHOOTER_POWER);
                    follower.setMaxPower(1);
                    follower.followPath(line5);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 6) {//(!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && (pathTimer.getElapsedTimeSeconds() < 7)) {
                        BeginShotSequence();


                    } else {

                        //out.block();
                        intake();
                        //outtakeBeamBreak.resetBallCount();
                        setPathState(7);
                    }
                }
                break;

                    case 13:
                        if (!follower.isBusy()) {
                            shooter.eStop();

                            follower.followPath(line6);

                            setPathState(14);
                        }
                        break;

                    case 14:
                        if (!follower.isBusy()) {
                            shooter.eStop();

                            follower.pausePathFollowing();
                        }
                        break;
                }
        }


    private void beginRun(ShotOrderPlanner planner, ColourZoneDetection.Snapshot
            snap, StaticShooter shooter, Kickers kickers) {
        plan = planner.plan(CIPHER, snap, FORCE_SHOOT_ALL_ZONES);
        stepIdx = 0;
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();
//        shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        shooterReadyTimer.reset();
        state = RunState.SPINUP_WAIT;
    }

    private void abortNow(StaticShooter shooter, Kickers kickers) {
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();
//        shooter.setTargetRPM(0.0);
        shooter.update();
        shooter.eStop();
        state = RunState.ABORTED;
        timer.reset();
    }

    private void runStateMachine(StaticShooter shooter, Kickers kickers) {
        if (state != RunState.IDLE && state != RunState.DONE && state != RunState.ABORTED) {
//            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        }
        switch (state) {
            case SPINUP_WAIT:
                if (plan == null || plan.isEmpty()) {
                    state = RunState.DONE;
                    return;
                }
                if (isShooterReady(shooter.getShooterVelocity()) || shooterReadyTimer.seconds() >= SHOOTER_READY_TIMEOUT_S) {
                    timer.reset();
                    state = RunState.KICK_UP;
                }
                break;
            case KICK_UP:
                if (stepIdx >= plan.size()) {
                    finishRun(shooter, kickers);
                    return;
                }
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
        }
    }

    private void finishRun(StaticShooter shooter, Kickers kickers) {
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();
//        shooter.setTargetRPM(0.0);
        state = RunState.DONE;
    }

    private boolean isShooterReady(double rpm) {
        return rpm >= (SHOOTER_TARGET_RPM - SHOOTER_READY_TOL_RPM) && rpm > 0.0;
    }

    private static void kickZone(Kickers k, ColourZoneDetection.ZoneId zone) {
        switch (zone) {
            case Z1:
                k.kickZoneOne();
                break;
            case Z2:
                k.kickZoneTwo();
                break;
            case Z3:
                k.kickZoneThree();
                break;
        }
    }


    private void BeginShotSequence() {
        beginRun(planner, snap, shooter, kickers);
    }

    private void intake () {
        intaking = true;
        intake.intakeEhub(0.8);
    }

    private void configurePinpoint() {
        pinpoint.resetPosAndIMU();
        pinpoint.setOffsets(145.12237, -145.12237, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }
}
