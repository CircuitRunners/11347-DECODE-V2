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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ServoTurretTracker; // NEW
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

@Config
@TeleOp(group = "1")
public class MainTeleOp extends CommandOpMode {

    // ---- Turret auto-aim ----
    public static boolean TURRET_AUTO_AIM = false;

    // Target pose (Pedro coords, inches)
    public static Pose TURRET_TARGET_POSE = new Pose(144, 144);
     public static double FIELD_SIZE_IN = 144.0;

     private double goalX_odom() {
         return FIELD_SIZE_IN - TURRET_TARGET_POSE.getX();
     }

     private double goalY_odom() {
         return FIELD_SIZE_IN - TURRET_TARGET_POSE.getY();
     }

    // Optional: turret trim (deg)
    public static double TURRET_TRIM_DEG = 0.0;

    // ---- Auto-sort / indexing ----
    public static ShotOrderPlanner.Cipher CIPHER = ShotOrderPlanner.Cipher.PPG;
    public static boolean START_SEQUENCE = false;
    public static boolean ABORT_SEQUENCE = false;
    public static boolean FORCE_SHOOT_ALL_ZONES = true;

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
    private GoBildaPinpointDriver pinpoint;

    // ===================== Software =====================
    private ShotOrderPlanner planner;
    private ColourZoneDetection czd;
    private GamepadEx driver, manipulator;

     // ===================== VISION =====================
     public static boolean VISION_FUSE_XY = false;
     public static double VISION_RATE_HZ = 5.0;
     public static double VISION_ALPHA = 0.25;
     public static double VISION_MAX_JUMP_IN = 18.0;
     public static double VISION_MAX_TX_DEG = 8.0;
     public static double VISION_MIN_DIST_IN = 8.0;        // reject ultra-close junk
     public static double VISION_MAX_DIST_IN = 200.0;      // reject far junk
     public static double TURRET_WINDOW_EDGE_DEG = 7.0;    // deadband from turret limits
     private double lastVisionX = Double.NaN;
     private double lastVisionY = Double.NaN;
     private double lastVisionUpdateS = 0.0;
     private String lastVisionReject = "none";
     private double txHoldDeg = Double.NaN;
     private double txHoldTimeS = 0.0;
     public static double TX_HOLD_SEC = 0.10; // 100ms

    // ===================== Dashboard telemetry =====================
    private FtcDashboard dash;

    // ===================== Auto-sort state machine =====================
    private enum RunState { IDLE, SPINUP_WAIT, KICK_UP, RESET_DOWN, PAUSE, DONE, ABORTED }
    private RunState state = RunState.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime shooterReadyTimer = new ElapsedTime();
    private List<ShotOrderPlanner.PlannedShot> plan = Collections.emptyList();
    private int stepIdx = 0;
    private boolean lastStart = false;
    //new stuff
    private Pose GOAL_POS_RED = new Pose(138,138);
    private double SCORE_HEIGHT = 25;
    private double SCORE_ANGLE = Math.toRadians(-30);
    private double PASS_THROUGH_POINT_RADIUS =5;
    public static  double HOOD_MAX_ANGLE = Math.toRadians(67);
    public static double HOOD_MIN_ANGLE = Math.toRadians(0);
    public static double wheelDiameter = 3.21;
    public static double maxHoodTicks = 0.8;
    private double hoodAngle = 0;
    private double flywheelSpeed = 0;
    private boolean usePhysics = true;
    private Follower follower;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

        dash = FtcDashboard.getInstance();
        dash.setTelemetryTransmissionInterval(25);
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        follower.update();

        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);

        // Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        // Subsystems
        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(000);

        limelight = new LimelightSubsystem(hardwareMap, "limelight");
        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.RED.getTagId());
        limelight.setAllianceTagID(AlliancePresets.getAllianceShooterTag());

        boolean isBlue = (AlliancePresets.getAllianceShooterTag() == AlliancePresets.Alliance.BLUE.getTagId());
        TURRET_TARGET_POSE = isBlue ? new Pose(0, 144) : new Pose(144, 144);

        drive = new MecanumDrivebase(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(hardwareMap);
        kickers = new Kickers(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        czd = new ColourZoneDetection(hardwareMap, "srsHubIndexer", "srsHubPlate");
        planner = new ShotOrderPlanner();

        // NEW turret tracker
        turret = new ServoTurretTracker(hardwareMap, "turret");
        turret.setEnabled(false);
        turret.setTargetFieldPointInches(TURRET_TARGET_POSE.getX(), TURRET_TARGET_POSE.getY());

        // Kicker reset
        kickers.resetZoneOne();
        kickers.resetZoneTwo();
        kickers.resetZoneThree();

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(()-> {
                    usePhysics = true;
                }));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(()-> {
                    usePhysics = true;
                }));



        // ===================== Commands =====================
        intake.setDefaultCommand(new IntakeCommand(intake, outtake, driver));

        // Driver toggle for turret auto-aim
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    TURRET_AUTO_AIM = !TURRET_AUTO_AIM;
                    outtake.setAutomatic(!outtake.isAutomatic());
                });

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()-> {
                    Pose2D pose = pinpoint.getPosition();
                    double headingOffset = pose.getHeading(AngleUnit.RADIANS);
                    pinpoint.setPosition(
                            new Pose2D(
                                    DistanceUnit.INCH,
                                    pose.getX(DistanceUnit.INCH),
                                    pose.getY(DistanceUnit.INCH),
                                    AngleUnit.RADIANS,
                                    Math.toRadians(180.0)
                            ));
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

                    follower.setPose(new Pose(72,72, Math.toRadians(0)));

                }));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> START_SEQUENCE = !START_SEQUENCE);

        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                72, 72,
                AngleUnit.RADIANS,
                Math.toRadians(180.0)
        ));

        telemetry.addLine("Init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        czd.update();
        shooter.update();
        limelight.update();
        pinpoint.update();

        shooter.setTargetRPM(SHOOTER_ENABLED ? SHOOTER_TARGET_RPM : 0.0);

        Pose2D currentPose = pinpoint.getPosition();
        double x = currentPose.getX(DistanceUnit.INCH);
        double y = currentPose.getY(DistanceUnit.INCH);

        // ===================== Drive =====================
        double forward = driver.getLeftY();
        double right = driver.getLeftX();
        double rotate = driver.getRightX();

        Pose2D drivePose;
        drivePose = driveFieldRelative(forward, right, rotate);

        calculateHoodPos(x, y);

        //new stuff
        double gearRatio = 1.0;

        double wheelRPM = MathFunctions.clamp((flywheelSpeed * 60.0) / (Math.PI * (wheelDiameter/4.0)), 0, 4000);
        double motorRPM = wheelRPM * gearRatio;

        double hoodPos = (maxHoodTicks - Range.scale(hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, 0.0, maxHoodTicks));

        if (usePhysics) {
            shooter.setTargetRPM(motorRPM);
            outtake.aimScoring(hoodPos);
        }

        // ===================== Turret Auto Aim =====================
        double gx = goalX_odom();
        double gy = goalY_odom();

//        double gx = TURRET_TARGET_POSE.getX();
//        double gy = TURRET_TARGET_POSE.getY();

        turret.setTargetFieldPointInches(gx, gy);
        ServoTurretTracker.TURRET_TRIM_DEG = TURRET_TRIM_DEG;
        turret.setEnabled(TURRET_AUTO_AIM);

        turret.update(currentPose);

//        // ===================== VISION =====================
//        double distLOS = limelight.getDistanceToTagCenterInches(false);
//        double distGround = limelight.getDistanceToTagCenterInches(true);
//        fuseXYFromLimelight(currentPose);

        // ===================== Auto-sort =====================
        ColourZoneDetection.Snapshot snap = czd.getStableSnapshot();
        if (ABORT_SEQUENCE) {
            ABORT_SEQUENCE = false;
            abortNow(shooter, kickers);
        }

        boolean startEdge = START_SEQUENCE && !lastStart;
        lastStart = START_SEQUENCE;

        if (startEdge && (state == RunState.IDLE || state == RunState.DONE || state == RunState.ABORTED)) {
            START_SEQUENCE = false;
            beginRun(planner, snap, shooter, kickers);
        }

        runStateMachine(shooter, kickers);

        // ===================== Intake =====================

        // ===================== OUTTAKE =====================
        Pose2D poseForHood = pinpoint.getPosition();
        double Hx = poseForHood.getX(DistanceUnit.INCH);
        double Hy = poseForHood.getY(DistanceUnit.INCH);

//        outtake.updateAutoHoodFromField(Hx, Hy, TURRET_TARGET_POSE.getX(), TURRET_TARGET_POSE.getY());
        outtake.updateAutoHoodFromField(Hx, Hy, gx, gy);
        outtake.update();

        // ===================== Telemetry =====================
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                drivePose.getX(DistanceUnit.INCH),
                drivePose.getY(DistanceUnit.INCH),
                drivePose.getHeading(AngleUnit.DEGREES)
        );
        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );
//        telemetry.addLine("----  Limelight Data  ----");
//        telemetry.addData("LL valid", limelight.hasValidTarget());
//        telemetry.addData("VisionReject", lastVisionReject);
//        telemetry.addData("Distance (LOS, in)", distLOS);
//        telemetry.addData("Distance (Ground, in)", distGround);
//        telemetry.addData("Tx", limelight.getTx());
//        telemetry.addData("Ty", limelight.getTy());
//        telemetry.addData("Vision X", lastVisionX);
//        telemetry.addData("Vision Y", lastVisionY);
//        telemetry.addLine();
        telemetry.addLine("---- TURRET ----");
        telemetry.addData("Turret AutoAim", TURRET_AUTO_AIM);
        telemetry.addData("Target Pose", "X:%.1f Y:%.1f", TURRET_TARGET_POSE.getX(), TURRET_TARGET_POSE.getY());
        telemetry.addData("RobotFieldHeading", "%.1f°", turret.getRobotFieldHeadingDeg());
        telemetry.addData("TargetFieldAngle", "%.1f°", turret.getTargetFieldDeg());
        telemetry.addData("TurretRobotCmd", "%.1f°", turret.getTurretRobotDegCmd());
        telemetry.addData("Turret ServoPos", "%.4f", turret.getServoPos());
        telemetry.addLine();
        telemetry.addLine("---- Hood Pose ----");
        telemetry.addData("Hood Pos", outtake.hoodPos());
        telemetry.addLine();
        telemetry.addLine("----  Pinpoint Data  ----");
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.addData("Goal Pedro", "x=%.1f y=%.1f", TURRET_TARGET_POSE.getX(), TURRET_TARGET_POSE.getY());
        telemetry.addData("Goal Odom",  "x=%.1f y=%.1f", goalX_odom(), goalY_odom());
        telemetry.addData("Robot Odom", "x=%.1f y=%.1f", currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Hood pos", hoodPos);
        telemetry.addData("Shooter Predicted Vel",motorRPM);
        telemetry.addData("Follower Position:", followerData);
        telemetry.addLine();

        telemetry.update();
        TelemetryPacket pDash = new TelemetryPacket();
        pDash.put("x", currentPose.getX(DistanceUnit.INCH));
        pDash.put("y", currentPose.getY(DistanceUnit.INCH));
        pDash.put("turretServoPos", turret.getServoPos());
        dash.sendTelemetryPacket(pDash);
    }

    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES) - 180);
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit
                .normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
        return pos;
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

     public void updateCoordinatesWithAprilTag() {
         limelight.limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.RADIANS));
         limelight.limelight.pipelineSwitch(0);
         LLResult result = limelight.limelight.getLatestResult();
         if (result != null && result.isValid()) {
             Pose3D mt1Pose = result.getBotpose();
             if (mt1Pose != null) {
                 double finalX = (mt1Pose.getPosition().y * 39.37) + 72.0;
                 double finalY = (-mt1Pose.getPosition().x * 39.37) + 72.0;
                 pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, finalX, finalY, AngleUnit.RADIANS, pinpoint.getHeading(AngleUnit.RADIANS)));
             }
         }
     }

     private static double wrap0to360(double deg) {
         deg %= 360.0;
         if (deg < 0) deg += 360.0;
         return deg;
     }

     private void fuseXYFromLimelight(Pose2D odomPose) {
         if (!VISION_FUSE_XY) { lastVisionReject = "disabled"; return; }

         double now = System.nanoTime() * 1e-9;
         double minPeriod = 1.0 / Math.max(0.1, VISION_RATE_HZ);
         if (now - lastVisionUpdateS < minPeriod) { lastVisionReject = "rate"; return; }

         // Must see the correct tag
         if (!limelight.hasValidTarget()) { lastVisionReject = "noTag"; return; }

         // Don’t trust when turret is near hard window edges (turret-mounted camera geometry/backlash)
         double turretDeg = turret.getTurretRobotDegCmd(); // last commanded turret robot-relative angle
         if (turretDeg <= ServoTurretTracker.TURRET_WINDOW_MIN_DEG + TURRET_WINDOW_EDGE_DEG) { lastVisionReject = "turretMin"; return; }
         if (turretDeg >= ServoTurretTracker.TURRET_WINDOW_MAX_DEG - TURRET_WINDOW_EDGE_DEG) { lastVisionReject = "turretMax"; return; }

         // Only fuse if tag is fairly centered in camera
         double tx = limelight.getTx(); // degrees
         if (Double.isNaN(tx) || Double.isInfinite(tx) || Math.abs(tx) > VISION_MAX_TX_DEG) { lastVisionReject = "txGate"; return; }

         // Distance to tag (ground plane)
         double dist = limelight.getDistanceToTagCenterInches(true);
         if (!(dist > 0.0) || Double.isNaN(dist) || Double.isInfinite(dist)) { lastVisionReject = "distNaN"; return; }
         if (dist < VISION_MIN_DIST_IN || dist > VISION_MAX_DIST_IN) { lastVisionReject = "distGate"; return; }

         double tagX = goalX_odom();
         double tagY = goalY_odom();

         // FIELD bearing robot->tag:
         // robotFieldHeadingDeg already includes alliance handling inside turret tracker.
         double robotFieldHeadingDeg = turret.getRobotFieldHeadingDeg();
         double bearingFieldDeg = wrap0to360(robotFieldHeadingDeg + turretDeg + tx);
         double bearingRad = Math.toRadians(bearingFieldDeg);

         // Robot position = tag position - dist * direction(robot->tag)
         double visionX = tagX - dist * Math.cos(bearingRad);
         double visionY = tagY - dist * Math.sin(bearingRad);

         lastVisionX = visionX;
         lastVisionY = visionY;

         // Reject huge jumps vs odom
         double ox = odomPose.getX(DistanceUnit.INCH);
         double oy = odomPose.getY(DistanceUnit.INCH);
         double dx = visionX - ox;
         double dy = visionY - oy;
         if (Math.hypot(dx, dy) > VISION_MAX_JUMP_IN) { lastVisionReject = "jump"; return; }

         // Blend into odom (keep Pinpoint heading)
         double fusedX = ox + VISION_ALPHA * dx;
         double fusedY = oy + VISION_ALPHA * dy;
         double h = odomPose.getHeading(AngleUnit.RADIANS);

         pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.RADIANS, h));

         lastVisionUpdateS = now;
         lastVisionReject = "ok";
     }

    private void beginRun(ShotOrderPlanner planner, ColourZoneDetection.Snapshot snap, StaticShooter shooter, Kickers kickers) {
        plan = planner.plan(CIPHER, snap, FORCE_SHOOT_ALL_ZONES);
        stepIdx = 0;
        kickers.resetZoneOne(); kickers.resetZoneTwo(); kickers.resetZoneThree();
//        shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        shooterReadyTimer.reset();
        state = RunState.SPINUP_WAIT;
    }

    private void abortNow(StaticShooter shooter, Kickers kickers) {
        kickers.resetZoneOne(); kickers.resetZoneTwo(); kickers.resetZoneThree();
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

    private void finishRun(StaticShooter shooter, Kickers kickers) {
        kickers.resetZoneOne(); kickers.resetZoneTwo(); kickers.resetZoneThree();
//        shooter.setTargetRPM(0.0);
        state = RunState.DONE;
    }

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

    public void calculateHoodPos(double robotX, double robotY) {
        // Horizontal distance to goal
        double dx = GOAL_POS_RED.getX() - robotX;
        double dy = GOAL_POS_RED.getY() - robotY;
        double distanceToGoal = Math.hypot(dx, dy);
        double angleToGoal = Math.atan(dy / dx);
        Vector robotToGoalVector = new Vector(distanceToGoal, angleToGoal);

        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        //calculuate initial launch components
        hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);

        flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));
}}
