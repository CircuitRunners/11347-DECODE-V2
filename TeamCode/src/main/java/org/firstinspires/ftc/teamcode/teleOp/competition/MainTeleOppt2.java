package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.util.Timer;
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
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import java.util.Locale;

@Config
@TeleOp(name="MainTeleOp", group="1")
public class MainTeleOppt2 extends CommandOpMode {
    // ============ Hardware ============
    private MecanumDrivebase drive;
    private StaticShooter shooter;
    private IntakeSubsystem intake;
    private Kickers kicker;

    // ============ Software ============
    private GamepadEx driver;
    private ColourZoneDetection czd;
    private final ShotOrderPlanner shotPlanner = new ShotOrderPlanner();
    private ShotOrderPlanner.Cipher cipher = ShotOrderPlanner.Cipher.PPG;
    private Timer loopTimer;
    private boolean alliance;
    private boolean isActive = false;

    public static boolean execute = false;

    // ============ Loop Time Stuff ============
    private long lastLoopNs = 0;
    private double loopMs = 0;
    private double avgLoopMs = 0;
    private double maxLoopMs = 0;
    private int loopCount = 0;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

        // ============ Hardware ============
        drive = new MecanumDrivebase(hardwareMap);

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        intake = new IntakeSubsystem(hardwareMap, telemetry);

        kicker = new Kickers(hardwareMap);

        // ============ Software ============
        driver = new GamepadEx(gamepad1);
        czd = new ColourZoneDetection(hardwareMap,
                "z1CSa", "z2CSa", "z3CSa",
                "z1CSb", "z2CSb", "z3CSb");

        loopTimer = new Timer();

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

        drive.setStaringPose2D();
        register(shooter, kicker);

        // ============ Telemetry ============
        telemetry.addLine("Init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        long nowNs = System.nanoTime();
        if (lastLoopNs != 0) {
            loopMs = (nowNs - lastLoopNs) / 1e6; // ms between loop starts
            loopCount++;
            avgLoopMs += (loopMs - avgLoopMs) / loopCount; // running average
            if (loopMs > maxLoopMs) maxLoopMs = loopMs;
        }
        lastLoopNs = nowNs;
        loopTimer.resetTimer();

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                drive.getPose().getX(DistanceUnit.INCH),
                drive.getPose().getY(DistanceUnit.INCH),
                drive.getPose().getHeading(AngleUnit.DEGREES)
        );

        telemetry.addData("loop dt (ms)", "%.3f", loopMs);
        telemetry.addData("loop avg (ms)", "%.3f", avgLoopMs);
        telemetry.addData("loop max (ms)", "%.3f", maxLoopMs);
        telemetry.addData("Pinpoint Looptimes", drive.getPinpointLooptime());
        telemetry.addData("Cipher", cipher);
        telemetry.addData("Position", data);
        telemetry.update();
    }
}
