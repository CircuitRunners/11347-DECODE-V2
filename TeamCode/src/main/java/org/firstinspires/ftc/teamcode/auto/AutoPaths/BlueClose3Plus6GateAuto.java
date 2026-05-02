package org.firstinspires.ftc.teamcode.auto.AutoPaths;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.AutoSortAndExecute;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakePivot;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.MotorTurretTracker;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;

@Config
@Autonomous(name = "Blue Side Auto Close 3+6 gate", group = "Blue Auto", preselectTeleOp = "v3MainTeleOpBlue")
public class BlueClose3Plus6GateAuto extends CommandOpMode {
    // ===================== ALLIANCE / FIELD =====================
    private final boolean isRed = false;
    private final Pose TARGET_GOAL_POSE = new Pose(-6, 134);
    private final Pose startPose = new Pose(19.5, 121.3, Math.toRadians(323.75));

    // ===================== HARDWARE =====================
    private IntakeSubsystem intake;
    private HoodSubsystem hood;
    private StaticShooter shooter;
    private Kickers kickers;
    private MotorTurretTracker turret;
    private Follower follower;
    private MecanumDrivebase drive;
    private IntakePivot pivot;

    // ========== SOFTWARE ==========
    private ColourZoneDetection czd;
    private final ShotOrderPlanner shotPlanner = new ShotOrderPlanner();
    private ShotOrderPlanner.Cipher cipher = ShotOrderPlanner.Cipher.PPG;
    private AutoSortAndExecute shootCommand = null;
    private boolean shootSequenceFinished = false;
    private ElapsedTime pathTimer;
    private ElapsedTime EndCode;

    // ===================== SHOOT SOLVER CONFIG =====================
    public static double PASS_THROUGH_RADIUS_IN = 0.5;
    public static double SCORE_HEIGHT_IN = 31;
    public static double SCORE_ANGLE_RAD = Math.toRadians(-30.0);
    public static double HOOD_MAX_ANGLE_RAD = Math.toRadians(67.0);
    public static double HOOD_MIN_ANGLE_RAD = Math.toRadians(0.0);
    public static double MAX_HOOD_TICKS = 0.96;

    // ===================== INTAKE =====================
    public static double INTAKE_POWER = 1.0;

    // ===================== AUTO STATE =====================
    private enum AutoState {
        START_FOLLOWING,
        PRELOAD_AIM_AND_SPINUP,
        PRELOAD_SHOOTING,
        DRIVE_COLLECT_AND_RETURN,
        SECOND_AIM_AND_SPINUP,
        SECOND_SHOOTING,
        DRIVE_TO_GATE,
        GATE,
        GATE_TO_SHOOTING,
        SHOOT_GATED_BALLS,
        DRIVE_LAST,
        CHECK_IF_DONE,
        DONE
    }
    private AutoState autoState = AutoState.PRELOAD_AIM_AND_SPINUP;
    private boolean startedOnce = false;
    private boolean NOT_AT_GATE = true;
    private boolean driveLast = false;
    private int gates = 0;

    // ===================== PATHS =====================
    private PathChain firstScoringPath, middleLineIntake, gateIntakePath, gateToScoring, lastPath;
    private void buildPaths() {
        firstScoringPath = follower.pathBuilder()
                // Start to Shoot 1 pose
                .addPath(
                        new BezierLine(
                                new Pose(19.1, 121.3),
                                new Pose(30, 112)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(323.75), Math.toRadians(290))
                .addPath(
                        new BezierLine(
                                new Pose(30, 112),
                                new Pose(54, 90)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(255))
                .build();

        middleLineIntake = follower.pathBuilder()
                // shoot 1 to middle line
                .addPath(
                        new BezierCurve(
                                new Pose(54, 90),
                                new Pose(66.7, 64),
                                new Pose(22, 60)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(255), Math.toRadians(180))
                // middle line to shoot pose
                .addPath(
                        new BezierCurve(
                                new Pose(22, 60),
                                new Pose(42, 72),
                                new Pose(54, 90)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                .build();

        gateIntakePath = follower.pathBuilder()
                // shoot pose to gate
                .addPath(
                        new BezierCurve(
                                new Pose(54, 90),
                                new Pose(46, 70),
                                new Pose(13, 59)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(155))
                .build();

        gateToScoring = follower.pathBuilder()
                // gate to shoot pose
                .addPath(
                        new BezierCurve(
                                new Pose(13, 59),
                                new Pose(46, 70),
                                new Pose(54, 90)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(225))
                .build();

        lastPath = follower.pathBuilder()
                // gate to shoot pose
                .addPath(
                        new BezierCurve(
                                new Pose(54, 90),
                                new Pose(51.5, 85),
                                new Pose(44, 84)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(44, 84),
                                new Pose(20, 84)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(20, 84),
                                new Pose(54, 106)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                .build();
    }

    private void setAutoState(AutoState newState) {
        autoState = newState;
    }

    private void updateTurretAim() {
        Pose robotPose = follower.getPose();

        Pose2D turretPose = new Pose2D(
                DistanceUnit.INCH,
                robotPose.getX(),
                robotPose.getY(),
                AngleUnit.RADIANS,
                robotPose.getHeading()
        );

        turret.setTargetFieldPointInches(TARGET_GOAL_POSE.getX(), TARGET_GOAL_POSE.getY());
        turret.updateAim(turretPose);
    }

    private void updateHoodAndShooter() {
        Pose p = follower.getPose();
        double robotX = p.getX();
        double robotY = p.getY();

        double dx = TARGET_GOAL_POSE.getX() - robotX;
        double dy = TARGET_GOAL_POSE.getY() - robotY;

        double distanceToGoal = Math.hypot(dx, dy);

        double g = 32.174 * 12.0;
        double distX = distanceToGoal - PASS_THROUGH_RADIUS_IN;
        double heightY = SCORE_HEIGHT_IN;
        double a = SCORE_ANGLE_RAD;

        double hoodAngle = Math.atan(2.0 * heightY / distX - Math.tan(a));
        hoodAngle = clamp(hoodAngle, HOOD_MIN_ANGLE_RAD, HOOD_MAX_ANGLE_RAD);

        double flywheelSpeed = Math.sqrt(
                g * distX * distX /
                        (2.0 * Math.pow(Math.cos(hoodAngle), 2.0) * (distX * Math.tan(hoodAngle) - heightY))
        );

        double gearRatio = 1.0;
        double wheelRPM = (flywheelSpeed * 60.0) / (Math.PI * (4.85 / 4.0));
        double motorRPM = wheelRPM * gearRatio;

        double hoodPos = (MAX_HOOD_TICKS - Range.scale(
                hoodAngle,
                HOOD_MIN_ANGLE_RAD,
                HOOD_MAX_ANGLE_RAD,
                0.0,
                0.9
        ));

        shooter.setTargetRPM(motorRPM);
        hood.aimScoring(hoodPos);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void initialize() {
        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        turret = new MotorTurretTracker(hardwareMap, isRed);
        turret.zeroAtStartupIfOnLimit();
        turret.setEnabled(false);

        intake = new IntakeSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap);
        kickers = new Kickers(hardwareMap);

        pivot = new IntakePivot(hardwareMap);

        czd = new ColourZoneDetection(hardwareMap,
                "z1CSa", "z2CSa", "z3CSa",
                "z1CSb", "z2CSb", "z3CSb");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        gates = 0;
        driveLast = false;

        pathTimer = new ElapsedTime();
        EndCode = new ElapsedTime();

        drive = new MecanumDrivebase(hardwareMap, false, true, follower);

        register(shooter, kickers, hood, turret);
        schedule(new BulkCacheCommand(hardwareMap));

        telemetry.addLine("BlueClose3Plus6GateAuto initialized");
        telemetry.addData("Turret homed", turret.isHomed());
        telemetry.update();
    }

    @Override
    public void run() {
        follower.update();

        if (!startedOnce) {
            pathTimer.reset();
            EndCode.reset();
            startedOnce = true;
            turret.setEnabled(true);
            kickers.resetZoneOne();
            kickers.resetZoneTwo();
            kickers.resetZoneThree();
            setAutoState(AutoState.START_FOLLOWING);
        }

        if (EndCode.seconds() > 29.5) {
            setAutoState(AutoState.DONE);
        }

        updateTurretAim();
        updateHoodAndShooter();

        switch (autoState) {
            case START_FOLLOWING:
                follower.setMaxPower(1);
                follower.followPath(firstScoringPath, false);
                setAutoState(AutoState.PRELOAD_AIM_AND_SPINUP);
                break;

            case PRELOAD_AIM_AND_SPINUP:
                if (pathTimer.seconds() > 1.2 && shooter.isAtTargetThreshold() || !follower.isBusy() && shooter.isAtTargetThreshold()) {
                    runShootCommand();
                    setAutoState(AutoState.PRELOAD_SHOOTING);
                }
                break;

            case PRELOAD_SHOOTING:
                if (shootSequenceFinished) {
                    follower.followPath(middleLineIntake, false);
                    intake.intake(INTAKE_POWER);
                    setAutoState(AutoState.DRIVE_COLLECT_AND_RETURN);
                }
                break;

            case DRIVE_COLLECT_AND_RETURN:
                intake.intake(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.stop();
                    setAutoState(AutoState.SECOND_AIM_AND_SPINUP);
                }
                break;

            case SECOND_AIM_AND_SPINUP:
                intake.intake(-INTAKE_POWER);
                if (shooter.isAtTargetThreshold()) {
                    runShootCommand();
                    setAutoState(AutoState.SECOND_SHOOTING);
                }
                break;

            case SECOND_SHOOTING:
                intake.stop();
                NOT_AT_GATE = true;
                if (shootSequenceFinished) {
                    if (gates >= 2) {
                        setAutoState(AutoState.DRIVE_LAST);
                    } else {
                        setAutoState(AutoState.DRIVE_TO_GATE);
                    }
                }
                break;

            case DRIVE_TO_GATE:
                follower.followPath(gateIntakePath, false);
                setAutoState(AutoState.GATE);
                break;

            case GATE:
                intake.intake(INTAKE_POWER);
                if (!follower.isBusy()) {
                    if (NOT_AT_GATE) {
                        pathTimer.reset();
                        NOT_AT_GATE = false;
                    }
                    if (pathTimer.seconds() > 0.5) {
                        setAutoState(AutoState.GATE_TO_SHOOTING);
                    }
                }
                break;

            case DRIVE_LAST:
                follower.followPath(lastPath, false);
                driveLast = true;
                setAutoState(AutoState.SHOOT_GATED_BALLS);
                break;

            case GATE_TO_SHOOTING:
                gates++;
                follower.followPath(gateToScoring, false);
                setAutoState(AutoState.SHOOT_GATED_BALLS);
                break;

            case SHOOT_GATED_BALLS:
                if (!follower.isBusy()) {
                    intake.intake(-INTAKE_POWER);
                    runShootCommand();
                    setAutoState(AutoState.CHECK_IF_DONE);
                }
                break;

            case CHECK_IF_DONE:
                if (shootSequenceFinished) {
                    if (!follower.isBusy()) {
                        if (gates >= 2 && driveLast) {
                            setAutoState(AutoState.DONE);
                        } else {
                            setAutoState(AutoState.SECOND_SHOOTING);
                        }
                    }
                }
                break;

            case DONE:
                turret.persistState();
                MecanumDrivebase.storeAutoPose(follower.getPose());
                intake.stop();
                shooter.setTargetRPM(0);
                follower.breakFollowing();
                break;
        }

        super.run();

        if (shootCommand != null) {
            shootSequenceFinished = shootCommand.isDoneRunning();
        }

        Pose robotPose = follower.getPose();

        telemetry.addData("Auto State", autoState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Pose", "x=%.2f y=%.2f h=%.1f",
                robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Turret Target Ticks", turret.getTargetTicks());
        telemetry.addData("Turret Cmd Deg", turret.getLastTurretHomeFrameDegCmd());
        telemetry.addData("Turret Measured Deg", turret.getLastTurretHomeFrameDegMeasured());
        telemetry.addData("Shooter RPM", shooter.getShooterVelocity());
        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
        telemetry.addData("Gates", gates);
        telemetry.update();
    }

    private void runShootCommand() {
        if (shootCommand != null && !shootCommand.isDoneRunning()) return;

        shootSequenceFinished = false;

        shootCommand = new AutoSortAndExecute(
                czd,
                kickers,
                shotPlanner,
                () -> cipher,
                0.04,
                0.01,
                false,
                true,
                telemetry
        );

        schedule(shootCommand);
    }
}
