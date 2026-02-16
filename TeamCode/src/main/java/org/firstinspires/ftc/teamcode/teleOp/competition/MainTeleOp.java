 package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ServoTurretTracker;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;
import org.firstinspires.ftc.teamcode.support.OdoAbsoluteHeadingTracking;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

@Config
@TeleOp(group = "1")
public class MainTeleOp extends CommandOpMode {
    public static Pose TURRET_TARGET_POSE = new Pose(0, 136);
    public static int redX = 137;
    public static int redY = 136;
    private final int blueX = -4;
    private final int blueY = 136;
    boolean isBlue = false;

    public static double TURRET_TRIM_DEG = 0.0;

    // ---- Auto-sort / indexing ----
    public static ShotOrderPlanner.Cipher CIPHER = ShotOrderPlanner.Cipher.PPG;
    public static boolean START_SEQUENCE = false;
    public static boolean ABORT_SEQUENCE = false;
    public static boolean FORCE_SHOOT_ALL_ZONES = false;

    public static double SHOOTER_TARGET_RPM = 3800.0;
    public static boolean SHOOTER_ENABLED = true;
    public static double SHOOTER_READY_TOL_RPM = 200.0;
    public static double SHOOTER_READY_TIMEOUT_S = 2.0;

    public static double KICK_UP_TIME_S = 0.30;
    public static double RESET_DOWN_TIME_S = 0.12;
    public static double BETWEEN_SHOTS_PAUSE_S = 0.15;

    // ===================== Hardware =====================
    private StaticShooter shooter;
    private Kickers kickers;
    private MecanumDrivebase drive;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private ServoTurretTracker turret;
    private LimelightSubsystem limelight;

    // ===================== Software =====================
    private ShotOrderPlanner planner;
    private ColourZoneDetection czd;
    private OdoAbsoluteHeadingTracking odoHeading;
    private GamepadEx driver, manipulator;
    private int x = 0;

    // ===================== Dashboard telemetry =====================
    private FtcDashboard dash;

    // ===================== Auto-sort state machine =====================
    private enum RunState { IDLE, SPINUP_WAIT, KICK_UP, RESET_DOWN, PAUSE, DONE, ABORTED }
    private RunState state = RunState.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime shooterReadyTimer = new ElapsedTime();
    private List<ShotOrderPlanner.PlannedShot> plan = Collections.emptyList();
    private int stepIdx = 0;

    private Pose GOAL_POS_RED = new Pose(138, 138); //138, 138
    public static double SCORE_HEIGHT = 25;
    private double SCORE_ANGLE = Math.toRadians(-30);
    private double PASS_THROUGH_POINT_RADIUS = 5;
    public static  double HOOD_MAX_ANGLE = Math.toRadians(67);
    public static double HOOD_MIN_ANGLE = Math.toRadians(0);
    public static double wheelDiameter = 4.8; //4.9
    private double hoodAngle = 0;
    private double flywheelSpeed = 0;
    private boolean usePhysics = true;
    private Follower follower;
    private Timer loopTimer;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

        dash = FtcDashboard.getInstance();
        dash.setTelemetryTransmissionInterval(25);
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);

        // Subsystems
        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        limelight = new LimelightSubsystem(hardwareMap, "limelight");
        limelight.setAllianceTagID(AlliancePresets.getAllianceShooterTag());

        isBlue = (AlliancePresets.getAllianceShooterTag() == AlliancePresets.Alliance.BLUE.getTagId());
        TURRET_TARGET_POSE = isBlue ? new Pose(blueX, blueY) : new Pose(redX, redY);

        drive = new MecanumDrivebase(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(hardwareMap);
        kickers = new Kickers(hardwareMap);
        odoHeading = new OdoAbsoluteHeadingTracking(
                intake.leftOdoMotor(),
                intake.rightOdoMotor()
        );

        follower = Constants.createFollower(hardwareMap);

        czd = new ColourZoneDetection(hardwareMap,
                "z1CSa", "z2CSa", "z3CSa",
                "z1CSb", "z2CSb", "z3CSb");        planner = new ShotOrderPlanner();
        turret = new ServoTurretTracker(hardwareMap, "turret");
        turret.setEnabled(false);
        turret.setTargetFieldPointInches(TURRET_TARGET_POSE.getX(), TURRET_TARGET_POSE.getY());

        loopTimer = new Timer();

        // Kicker reset
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(()-> usePhysics = !usePhysics));

        // ===================== Commands =====================
        intake.setDefaultCommand(new IntakeCommand(intake, driver));

        // Driver toggle for turret auto-aim
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    turret.setEnabled(!turret.isEnabled());
                    outtake.setAutomatic(!outtake.isAutomatic());
                });

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()-> {
                    Pose pose = follower.getPose();
                    follower.setPose(new Pose(
                            pose.getX(),
                            pose.getY(),
                            0.0
                    ));
                    odoHeading.reset(0.0);
                }));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whileHeld(new InstantCommand(()->
                            outtake.aiming(true, false)
                    ));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whileHeld(new InstantCommand(()->
                            outtake.aiming(false, true)
                    ));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(()-> {
                    follower.setPose(new Pose(
                            72,
                            72,
                            0.0
                    ));
                    odoHeading.reset(0.0);
                }));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(()-> x++));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> START_SEQUENCE = true);

        if (AlliancePresets.getGlobalPose() != null) {
            follower.setPose(AlliancePresets.getGlobalPose());
            odoHeading.reset(0.0);
        } else {
            follower.setPose(new Pose(
                    72,
                    72,
                    0.0
            ));
            odoHeading.reset(0.0);
        }

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(new InstantCommand(()-> {
                            if (isBlue) {
                                TURRET_TARGET_POSE = new Pose(redX, redY);
                            } else {
                                TURRET_TARGET_POSE = new Pose(blueX, blueY);
                            }
                            isBlue = !isBlue;
                        }));

        register(limelight, shooter, czd, odoHeading);
        telemetry.addLine("Init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        loopTimer.resetTimer();
        driver.readButtons();
        manipulator.readButtons();
        follower.update();
        cypher();

        // ===================== Auto-sort =====================
        shooter.setTargetRPM(SHOOTER_ENABLED ? SHOOTER_TARGET_RPM : 0.0);
        ColourZoneDetection.Snapshot snap = czd.getStableSnapshot();
        if (ABORT_SEQUENCE) {
            ABORT_SEQUENCE = false;
            abortNow(shooter, kickers);
        }

        if (START_SEQUENCE && (state == RunState.IDLE || state == RunState.DONE || state == RunState.ABORTED)) {
            START_SEQUENCE = false;
            beginRun(planner, snap, shooter, kickers);
        }

        runStateMachine(shooter, kickers);

        // Field Pose
        Pose currentPose = follower.getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();
        double heading = odoHeading.getHeadingRad();

        calculateHoodPos(x, y, heading, follower.getVelocity());

        double gearRatio = 1.0;

        double wheelRPM = (flywheelSpeed * 60.0) / (Math.PI * (wheelDiameter / 4.0));
        double motorRPM = wheelRPM * gearRatio;

        if (usePhysics) {
            shooter.setTargetRPM(motorRPM);
        }

        // ===================== Drive =====================
        double forward = driver.getLeftY();
        double right = driver.getLeftX();
        double rotate = driver.getRightX();

        Pose drivePose;
        drivePose = driveFieldRelative(forward, right, rotate);

        // ===================== Turret Auto Aim =====================
        double gx = TURRET_TARGET_POSE.getX();
        double gy = TURRET_TARGET_POSE.getY();

        turret.setTargetFieldPointInches(gx, gy);
        ServoTurretTracker.TURRET_TRIM_DEG = TURRET_TRIM_DEG;

        follower.update();

        Pose2D turretPose = new Pose2D(
                DistanceUnit.INCH,
                follower.getPose().getX(),
                follower.getPose().getY(),
                AngleUnit.RADIANS,
                odoHeading.getHeadingRad()
        );

        turret.update(turretPose);

        // ===================== OUTTAKE =====================
        Pose2D poseForHood = new Pose2D(
                DistanceUnit.INCH,
                follower.getPose().getX(),
                follower.getPose().getY(),
                AngleUnit.RADIANS,
                odoHeading.getHeadingRad()
        );

        double Hx = poseForHood.getX(DistanceUnit.INCH);
        double Hy = poseForHood.getY(DistanceUnit.INCH);

        outtake.updateAutoHoodFromField(Hx, Hy, gx, gy);
        outtake.update();

        // ===================== Telemetry =====================
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                drivePose.getX(),
                drivePose.getY(),
                drivePose.getHeading()
        );
        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );
        telemetry.addLine("---- LOOP TIMER ----");
        telemetry.addData("loop time",loopTimer.getElapsedTime());
        telemetry.addLine();
        telemetry.addLine("---- TURRET ----");
        telemetry.addData("Turret AutoAim", turret.isEnabled());
        telemetry.addData("Target Pose", "X:%.1f Y:%.1f", TURRET_TARGET_POSE.getX(), TURRET_TARGET_POSE.getY());
        telemetry.addData("RobotFieldHeading", "%.1f°", turret.getRobotFieldHeadingDeg());
        telemetry.addData("TargetFieldAngle", "%.1f°", turret.getTargetFieldDeg());
        telemetry.addData("TurretRobotCmd", "%.1f°", turret.getTurretRobotDegCmd());
        telemetry.addData("Turret ServoPos", "%.4f", turret.getServoPos());
        telemetry.addData("Current Cypher", CIPHER);
        telemetry.addLine();
        telemetry.addLine("---- Hood Pose ----");
        telemetry.addData("Hood Pos", outtake.getHoodPos());
        telemetry.addLine();
        telemetry.addLine("----  Pinpoint Data  ----");
        telemetry.addData("Position", data);
        telemetry.addData("Goal Pedro", "x=%.1f y=%.1f", TURRET_TARGET_POSE.getX(), TURRET_TARGET_POSE.getY());
        telemetry.addData("Robot Odom", "x=%.1f y=%.1f", currentPose.getX(), currentPose.getY());
        telemetry.addData("AbsoOdo Heading (deg)", odoHeading.getHeadingDeg());
        telemetry.addData("AbsoOdo Heading (rad)", odoHeading.getHeadingRad());
        telemetry.addData("Follower Position:", followerData);
        telemetry.addData("Shooter Predicted Vel",motorRPM);
        telemetry.addData("Robot velocity", follower.getVelocity().getMagnitude());
        telemetry.addLine();

        TelemetryPacket pDash = new TelemetryPacket();
        pDash.put("x", currentPose.getX());
        pDash.put("y", currentPose.getY());
        pDash.put("turretServoPos", turret.getServoPos());
        dash.sendTelemetryPacket(pDash);
        telemetry.update();
    }

    private Pose driveFieldRelative(double forward, double right, double rotate) {
        Pose pos = follower.getPose();  // Current position
        double robotAngle = 0;

        if (isBlue) {
            robotAngle = (pos.getHeading() - Math.toRadians(180));
        } else {
            robotAngle = pos.getHeading();
        }

        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit
                .normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
        return pos;
    }

    private void beginRun(ShotOrderPlanner planner, ColourZoneDetection.Snapshot snap, StaticShooter shooter, Kickers kickers) {
        plan = planner.plan(CIPHER, snap, FORCE_SHOOT_ALL_ZONES);
        stepIdx = 0;
        kickers.resetZoneOne(); kickers.resetZoneTwo(); kickers.resetZoneThree();
        shooterReadyTimer.reset();
        state = RunState.SPINUP_WAIT;
    }

    private void abortNow(StaticShooter shooter, Kickers kickers) {
        kickers.resetZoneOne(); kickers.resetZoneTwo(); kickers.resetZoneThree();
        shooter.update();
        shooter.eStop();
        state = RunState.ABORTED;
        timer.reset();
    }

    private void runStateMachine(StaticShooter shooter, Kickers kickers) {
        switch (state) {
            case SPINUP_WAIT:
                if (plan == null || plan.isEmpty()) { state = RunState.DONE; return; }
                if (isShooterReady(shooter.getShooterVelocity()) || shooterReadyTimer.seconds() >= SHOOTER_READY_TIMEOUT_S) {
                    timer.reset(); state = RunState.KICK_UP;
                }
                break;
            case KICK_UP:
                if (stepIdx >= plan.size()) { finishRun(shooter, kickers); return; }
                kickZone(kickers, plan.get(stepIdx).getZone());
                if (timer.seconds() >= KICK_UP_TIME_S) {
                    kickers.resetZoneOne(); kickers.resetZoneTwo(); kickers.resetZoneThree();
                    timer.reset(); state = RunState.RESET_DOWN;
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
                if (timer.seconds() >= BETWEEN_SHOTS_PAUSE_S) { timer.reset(); state = RunState.KICK_UP; }
                break;
        }
    }

    private void cypher(){
        switch (x){
            case 1:
                CIPHER = ShotOrderPlanner.Cipher.GPP;
                break;
            case 2:
                CIPHER = ShotOrderPlanner.Cipher.PGP;
                break;
            case 3:
                CIPHER = ShotOrderPlanner.Cipher.PPG;
                x = 0;
                break;

        }
    }

    private void finishRun(StaticShooter shooter, Kickers kickers) {
        kickers.resetZoneOne(); kickers.resetZoneTwo(); kickers.resetZoneThree();
        state = RunState.DONE;
    }

    private boolean isShooterReady(double rpm) {
        return shooter.isAtTargetThreshold();
    }

    private static void kickZone(Kickers k, ColourZoneDetection.ZoneId zone) {
        switch (zone) {
            case Z1: k.kickZoneOne(); break;
            case Z2: k.kickZoneTwo(); break;
            case Z3: k.kickZoneThree(); break;
        }
    }

    public void calculateHoodPos(double robotX, double robotY, double robotHeading, Vector robotVelocity) {
        // Horizontal distance to goal
        double dx = GOAL_POS_RED.getX() - robotX;
        double dy = GOAL_POS_RED.getY() - robotY;
        double distanceToGoal = Math.hypot(dx, dy);
        double angleToGoal = Math.atan2(dy, dx);
        Vector robotToGoalVector = new Vector(distanceToGoal, angleToGoal);

        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;

        double a = SCORE_ANGLE;

        //calculuate initial launch components
        hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);

        flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));


        //get robot velocity and convert it into parallel and perpendicular components
        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        //velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        //recalculuate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));
    }
}
