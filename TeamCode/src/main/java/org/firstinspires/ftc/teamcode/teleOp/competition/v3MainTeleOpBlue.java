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
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShotOrderPlanner;
import org.firstinspires.ftc.teamcode.commands.TurretAutoAim;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.MotorTurretTracker;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;

import java.util.Locale;

@TeleOp
@Config
public class v3MainTeleOpBlue extends CommandOpMode {
    // ========== HARDWARE ==========
    private MecanumDrivebase drive;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;
    private StaticShooter shooter;
    private Kickers kickers;
    private MotorTurretTracker turret;

    // ========== SOFTWARE ==========
    private ColourZoneDetection czd;
    private final ShotOrderPlanner shotPlanner = new ShotOrderPlanner();
    private ShotOrderPlanner.Cipher cipher = ShotOrderPlanner.Cipher.PPG;
    private boolean isRed = false;
    private boolean firstRun = true;
    private final Pose TARGET_GOAL_POSE = new Pose(-6, 134);
    private final static double SCORE_ANGLE = Math.toRadians(-30);
    private final static double HOOD_MAX_ANGLE = Math.toRadians(67);
    private final static double HOOD_MIN_ANGLE = Math.toRadians(0);
    private GamepadEx driver;

    // ============ Loop Time Stuff ============
    private long lastLoopNs = 0;
    private double loopMs = 0;
    private double avgLoopMs = 0;
    private double maxLoopMs = 0;
    private int loopCount = 0;

    @Override
    public void initialize() {
        // ========== HARDWARE ==========
        drive = new MecanumDrivebase(hardwareMap, isRed, false);

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        turret = new MotorTurretTracker(hardwareMap, isRed);
        turret.zeroAtStartupIfOnLimit();
        turret.setEnabled(false);

        intake = new IntakeSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap);

        kickers = new Kickers(hardwareMap);

        // ========== SOFTWARE ==========
        driver = new GamepadEx(gamepad1);
        czd = new ColourZoneDetection(hardwareMap,
                "z1CSa", "z2CSa", "z3CSa",
                "z1CSb", "z2CSb", "z3CSb");

        // ========== COMMANDS ==========
        drive.setDefaultCommand(new DriveCommand(drive, driver, isRed));
        intake.setDefaultCommand(new IntakeCommand(intake, driver));

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()-> {
                    drive.setPose(new Pose(drive.getPose().getX(), drive.getPose().getY(), Math.toRadians(0.0)));
                }));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new AutoSortAndExecute(
                        czd,
                        kickers,
                        shotPlanner,
                        () -> cipher,
                        0.04,
                        0.01,
                        false,
                        true,
                        telemetry
                ));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new AutoSortAndExecute(
                        czd,
                        kickers,
                        shotPlanner,
                        () -> cipher,
                        0.12,
                        0.12,
                        false,
                        false,
                        telemetry
                ));

        // ========== REGISTER ==========
        register(shooter, kickers, hood, turret);
        schedule(new BulkCacheCommand(hardwareMap),
                new PerpetualCommand(
                        new CalculateHoodPoseAndVelocity(
                                drive, shooter, hood, TARGET_GOAL_POSE,
                                0.5, 20.0,
                                SCORE_ANGLE, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE
                        )),
                new PerpetualCommand(
                        new TurretAutoAim(
                                drive, turret, TARGET_GOAL_POSE, isRed
                        ))
        );

        // ========== SET START POSE ==========


        // ========== TELEMETRY ==========
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

        if (firstRun) {
            turret.setEnabled(true);
            firstRun = !firstRun;
        }

        telemetry.addData("loop dt (ms)", "%.3f", loopMs);
        telemetry.addData("loop avg (ms)", "%.3f", avgLoopMs);
        telemetry.addData("loop max (ms)", "%.3f", maxLoopMs);
        telemetry.addData("Turret Target", turret.getTargetTicks());
        telemetry.addData("Cipher", cipher);
        telemetry.addData("Position", data);
        telemetry.update();
    }
}
