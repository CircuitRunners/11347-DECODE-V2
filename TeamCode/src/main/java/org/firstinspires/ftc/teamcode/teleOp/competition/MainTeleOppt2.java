package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.AutoSortAndExecute;
import org.firstinspires.ftc.teamcode.commands.CalculateHoodPoseAndVelocity;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.commands.TurretAutoAim;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ServoTurretTracker;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;
import org.firstinspires.ftc.teamcode.support.GobildaRGBIndicatorHelper;
import org.firstinspires.ftc.teamcode.support.OdoAbsoluteHeadingTracking;

import java.util.Locale;

@Config
@TeleOp(name="MainTeleOp", group="1")
public class MainTeleOppt2 extends CommandOpMode {
    // ============ Hardware ============
    private MecanumDrivebase drive;
    private StaticShooter shooter;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;
    private Kickers kicker;
    private ServoTurretTracker turret;

    // ============ Software ============
    private GamepadEx driver;
    private ColourZoneDetection czd;
    private GobildaRGBIndicatorHelper rgb;
    private OdoAbsoluteHeadingTracking odoHeading;
    private final ShotOrderPlanner shotPlanner = new ShotOrderPlanner();
    private ShotOrderPlanner.Cipher cipher = ShotOrderPlanner.Cipher.PPG;
    private boolean isRed;
    private boolean isActive = false;

    // ============ Loop Time Stuff ============
    private long lastLoopNs = 0;
    private double loopMs = 0;
    private double avgLoopMs = 0;
    private double maxLoopMs = 0;
    private int loopCount = 0;

    // ============ Physics ============ 
    private final Pose GOAL_POS_RED = new Pose(138, 138); //138, 138
    private final Pose GOAL_POS_BLUE = new Pose(-4, 136);
    private final double SCORE_ANGLE = Math.toRadians(-30);
    public static double HOOD_MAX_ANGLE = Math.toRadians(67);
    public static double HOOD_MIN_ANGLE = Math.toRadians(0);

    @Override
    public void initialize() {
        // ============ Hardware ============
        drive = new MecanumDrivebase(hardwareMap, true);

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        intake = new IntakeSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap);

        kicker = new Kickers(hardwareMap);

        turret = new ServoTurretTracker(hardwareMap, "turret");

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
        isRed = AlliancePresets.getAllianceShooterTag() == AlliancePresets.Alliance.RED.getTagId();

        Pose TARGET_GOAL_POSE = isRed ? GOAL_POS_RED : GOAL_POS_BLUE;

        // ============ Commands ============
        drive.setDefaultCommand(new DriveCommand(drive, driver, isRed));
        intake.setDefaultCommand(new IntakeCommand(intake, driver, czd, rgb));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    isActive = !isActive;
                    shooter.setTargetRPM(isActive ? 2500 : 0);
                }));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(()->
                        drive.setPose(new Pose(72,72, Math.toRadians(0)))
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
                        false,
                        telemetry
                ));

        // ============ Registered & Auto Updating Code ============
        register(shooter, kicker, hood);
        schedule(new BulkCacheCommand(hardwareMap),
                new PerpetualCommand(
                        new CalculateHoodPoseAndVelocity(
                                drive, shooter, hood, TARGET_GOAL_POSE,
                                5.0, 20.0, SCORE_ANGLE,
                                HOOD_MIN_ANGLE, HOOD_MAX_ANGLE
                        )),
                new PerpetualCommand(
                        new TurretAutoAim(
                                drive, turret, odoHeading, TARGET_GOAL_POSE
                        ))
        );

        // ============ Set Start Pose ============
        drive.setStartingPose();
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

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                drive.getPose().getX(),
                drive.getPose().getY(),
                drive.getPose().getHeading()
        );

        telemetry.addData("loop dt (ms)", "%.3f", loopMs);
        telemetry.addData("loop avg (ms)", "%.3f", avgLoopMs);
        telemetry.addData("loop max (ms)", "%.3f", maxLoopMs);
        telemetry.addData("Cipher", cipher);
        telemetry.addData("Position", data);
        telemetry.update();
    }
}
