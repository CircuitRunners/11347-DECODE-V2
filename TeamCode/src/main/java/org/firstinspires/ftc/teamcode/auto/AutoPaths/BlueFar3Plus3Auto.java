package org.firstinspires.ftc.teamcode.auto.AutoPaths;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.AutoSortAndExecute;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.MotorTurretTracker;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;

@Config
@Autonomous(name = "Blue Side Auto Far 3+3", group = "Blue Auto", preselectTeleOp = "v3MainTeleOpBlue")
public class BlueFar3Plus3Auto extends CommandOpMode {

    // ===================== ALLIANCE / FIELD =====================
    private final boolean isRed = false;
    private final Pose TARGET_GOAL_POSE = new Pose(-6, 126);
    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));

    // ===================== HARDWARE =====================
    private IntakeSubsystem intake;
    private HoodSubsystem hood;
    private StaticShooter shooter;
    private Kickers kickers;
    private MotorTurretTracker turret;
    private Follower follower;
    private MecanumDrivebase drive;

    // ===================== PATHS =====================
    private PathChain collectAndReturnPath;

    // ========== SOFTWARE ==========
    private ColourZoneDetection czd;
    private final ShotOrderPlanner shotPlanner = new ShotOrderPlanner();
    private ShotOrderPlanner.Cipher cipher = ShotOrderPlanner.Cipher.PPG;
    private AutoSortAndExecute shootCommand = null;
    private boolean shootSequenceFinished = false;

    // ===================== SHOOT SOLVER CONFIG =====================
    public static double PASS_THROUGH_RADIUS_IN = 0.5;
    public static double SCORE_HEIGHT_IN = 20.0;
    public static double SCORE_ANGLE_RAD = Math.toRadians(-30.0);
    public static double HOOD_MAX_ANGLE_RAD = Math.toRadians(67.0);
    public static double HOOD_MIN_ANGLE_RAD = Math.toRadians(0.0);
    public static double MAX_HOOD_TICKS = 0.96;

    // ===================== INTAKE =====================
    public static double INTAKE_POWER = 1.0;

    // ===================== AUTO STATE =====================
    private enum AutoState {
        PRELOAD_AIM_AND_SPINUP,
        PRELOAD_SHOOTING,
        DRIVE_COLLECT_AND_RETURN,
        SECOND_AIM_AND_SPINUP,
        SECOND_SHOOTING,
        DONE
    }
    private AutoState autoState = AutoState.PRELOAD_AIM_AND_SPINUP;
    private boolean startedOnce = false;

    private void buildPaths() {
        collectAndReturnPath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57, 9),
                                new Pose(55.5, 38),
                                new Pose(16, 35.5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(16, 35.5),
                                new Pose(50, 15)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
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
                0.05,
                0.8
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

        czd = new ColourZoneDetection(hardwareMap,
                "z1CSa", "z2CSa", "z3CSa",
                "z1CSb", "z2CSb", "z3CSb");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        drive = new MecanumDrivebase(hardwareMap, false, true, follower);

        register(shooter, kickers, hood, turret);
        schedule(new BulkCacheCommand(hardwareMap));

        telemetry.addLine("BlueFar3Plus3FastAuto initialized");
        telemetry.addData("Turret homed", turret.isHomed());
        telemetry.update();
    }

    @Override
    public void run() {
        follower.update();

        if (!startedOnce) {
            startedOnce = true;
            turret.setEnabled(true);
            kickers.resetZoneOne();
            kickers.resetZoneTwo();
            kickers.resetZoneThree();
            setAutoState(AutoState.PRELOAD_AIM_AND_SPINUP);
        }

        updateTurretAim();
        updateHoodAndShooter();

        switch (autoState) {
            case PRELOAD_AIM_AND_SPINUP:
                intake.stop();
                if (shooter.isAtTargetThreshold()) {
                    runShootCommand();
                    setAutoState(AutoState.PRELOAD_SHOOTING);
                }
                break;

            case PRELOAD_SHOOTING:
                follower.setMaxPower(0.8);
                intake.stop();
                if (shootSequenceFinished) {
                    follower.followPath(collectAndReturnPath, false);
                    intake.intake(INTAKE_POWER);
                    setAutoState(AutoState.DRIVE_COLLECT_AND_RETURN);
                }
                break;

            case DRIVE_COLLECT_AND_RETURN:
                intake.intake(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.stop();
                    kickers.resetZoneOne();
                    kickers.resetZoneTwo();
                    kickers.resetZoneThree();
                    setAutoState(AutoState.SECOND_AIM_AND_SPINUP);
                }
                break;

            case SECOND_AIM_AND_SPINUP:
                follower.setMaxPower(1);
                intake.stop();
                if (shooter.isAtTargetThreshold()) {
                    runShootCommand();
                    setAutoState(AutoState.SECOND_SHOOTING);
                }
                break;

            case SECOND_SHOOTING:
                intake.stop();
                if (shootSequenceFinished) {
                    setAutoState(AutoState.DONE);
                }
                break;

            case DONE:
                intake.stop();
                shooter.setTargetRPM(0);
                turret.persistState();
                MecanumDrivebase.storeAutoPose(follower.getPose());
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
