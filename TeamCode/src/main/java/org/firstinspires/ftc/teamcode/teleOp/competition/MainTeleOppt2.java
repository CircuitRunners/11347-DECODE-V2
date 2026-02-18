package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.AutoSortAndExecute;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;
import org.firstinspires.ftc.teamcode.support.GobildaRGBIndicatorHelper;
import org.firstinspires.ftc.teamcode.support.OdoAbsoluteHeadingTracking;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

@Config
@TeleOp(name="MainTeleOp", group="1")
public class MainTeleOppt2 extends CommandOpMode {
    // ============ Hardware ============
    private MecanumDrivebase drive;
    private StaticShooter shooter;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private Kickers kicker;

    // ============ Software ============
    private GamepadEx driver;
    private ColourZoneDetection czd;
    private GobildaRGBIndicatorHelper rgb;
    private OdoAbsoluteHeadingTracking odoHeading;
    private final ShotOrderPlanner shotPlanner = new ShotOrderPlanner();
    private ShotOrderPlanner.Cipher cipher = ShotOrderPlanner.Cipher.PPG;
    private boolean alliance;
    private boolean isActive = false;

    // ============ Loop Time Stuff ============
    private long lastLoopNs = 0;
    private double loopMs = 0;
    private double avgLoopMs = 0;
    private double maxLoopMs = 0;
    private int loopCount = 0;

    // ============ Ball Detection Stuff ============

    // ============ Physics ============ 
    private Pose GOAL_POS_RED = new Pose(138, 138); //138, 138
    public static double SCORE_HEIGHT = 20; //25
    private double SCORE_ANGLE = Math.toRadians(-30);
    private double PASS_THROUGH_POINT_RADIUS = 5;
    public static double HOOD_MAX_ANGLE = Math.toRadians(67);
    public static double HOOD_MIN_ANGLE = Math.toRadians(0);
    public static double maxHoodTicks = 0.90;
    public static double wheelDiameter = 4.8; //4.9
    private double hoodAngle = 0;
    private double flywheelSpeed = 0;
    private boolean usePhysics = true;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

        // ============ Hardware ============
        drive = new MecanumDrivebase(hardwareMap);

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        intake = new IntakeSubsystem(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(hardwareMap);

        kicker = new Kickers(hardwareMap);

        // ============ Software ============
        driver = new GamepadEx(gamepad1);
        czd = new ColourZoneDetection(hardwareMap,
                "z1CSa", "z2CSa", "z3CSa",
                "z1CSb", "z2CSb", "z3CSb");
        rgb = new GobildaRGBIndicatorHelper(hardwareMap);
        odoHeading = new OdoAbsoluteHeadingTracking(
                intake.leftOdoMotor(),
                intake.rightOdoMotor()
        );

        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.RED.getTagId());
        alliance = AlliancePresets.getAllianceShooterTag() == AlliancePresets.Alliance.BLUE.getTagId();

        // ============ Commands ============
        drive.setDefaultCommand(new DriveCommand(drive, driver, alliance));
        intake.setDefaultCommand(new IntakeCommand(intake, driver));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    isActive = !isActive;
                    shooter.setTargetRPM(isActive ? 2500 : 0);
                }));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(()->
                        drive.setPose2D(new Pose2D(DistanceUnit.INCH, 72,72, AngleUnit.RADIANS,Math.toRadians(0)))
                ));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new AutoSortAndExecute(
                        czd,
                        kicker,
                        shotPlanner,
                        () -> cipher,
                        0.18,
                        0.12,
                        false,
                        false
                ));

        // ============ Registered & Auto Updating Code ============
        register(shooter, kicker);

        // ============ Set Start Pose ============
        drive.setStaringPose2D();
        rgb.setColour(GobildaRGBIndicatorHelper.Colour.GREEN);

        // ============ Telemetry ============
        telemetry.addLine("Init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        long nowNs = System.nanoTime();
        if (lastLoopNs != 0) {
            loopMs = (nowNs - lastLoopNs) / 1e6;
            loopCount++;
            avgLoopMs += (loopMs - avgLoopMs) / loopCount;
            if (loopMs > maxLoopMs) maxLoopMs = loopMs;
        }
        lastLoopNs = nowNs;

        super.run();
        outtake.update();

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                drive.getPose().getX(DistanceUnit.INCH),
                drive.getPose().getY(DistanceUnit.INCH),
                drive.getPose().getHeading(AngleUnit.DEGREES)
        );

        rgb.setColour(GobildaRGBIndicatorHelper.Colour.RED);

        Pose2D currentPose = drive.getPose();
        double x = currentPose.getX(DistanceUnit.INCH);
        double y = currentPose.getY(DistanceUnit.INCH);
        double heading = odoHeading.getHeadingRad();

        calculateHoodPos(x, y, heading, drive.getVelocity());

        double gearRatio = 1.0;

        double wheelRPM = (flywheelSpeed * 60.0) / (Math.PI * (wheelDiameter / 4.0));
        double motorRPM = wheelRPM * gearRatio;

        double hoodPos = (maxHoodTicks - Range.scale(hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, 0.05, 0.8));

        if (usePhysics) {
            shooter.setTargetRPM(motorRPM);
            outtake.aimScoring(hoodPos);
        }

        telemetry.addData("loop dt (ms)", "%.3f", loopMs);
        telemetry.addData("loop avg (ms)", "%.3f", avgLoopMs);
        telemetry.addData("loop max (ms)", "%.3f", maxLoopMs);
        telemetry.addData("Pinpoint Looptimes", drive.getPinpointLooptime());
        telemetry.addData("Cipher", cipher);
        telemetry.addData("Position", data);
        telemetry.addData("Hood pos", hoodPos);
        telemetry.addData("Shooter Predicted Vel",motorRPM);
        telemetry.update();
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

//
//        //get robot velocity and convert it into parallel and perpendicular components
//        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
//
//        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
//        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();
//
//        //velocity compensation variables
//        double vz = flywheelSpeed * Math.sin(hoodAngle);
//        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
//        double ivr = x / time + parallelComponent;
//        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
//        double ndr = nvr * time;
//
//        //recalculuate launch components
//        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
//
//        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));
    }
}
