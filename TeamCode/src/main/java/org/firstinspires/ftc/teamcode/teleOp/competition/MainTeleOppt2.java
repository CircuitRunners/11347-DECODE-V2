package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import java.util.Locale;

@TeleOp(name="MainTeleOp", group="~")
public class MainTeleOppt2 extends CommandOpMode {
    // ============ Hardware ============
    private MecanumDrivebase drive;
    private StaticShooter shooter;
    private IntakeSubsystem intake;

    // ============ Software ============
    private GamepadEx driver;
    private Timer loopTimer;
    private boolean alliance;
    private boolean isActive = false;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

        // ============ Hardware ============
        drive = new MecanumDrivebase(hardwareMap);
        drive.setStaringPose2D();

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        intake = new IntakeSubsystem(hardwareMap, telemetry);

        // ============ Software ============
        driver = new GamepadEx(gamepad1);

        loopTimer = new Timer();

        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.RED.getTagId());
        alliance = AlliancePresets.getAllianceShooterTag() == AlliancePresets.Alliance.BLUE.getTagId();

        // ============ Commands ============
        drive.setDefaultCommand(new DriveCommand(drive, driver, alliance));
        intake.setDefaultCommand(new IntakeCommand(intake, driver));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    isActive = !isActive;
                    shooter.setTargetRPM(isActive ? 2500 : 0);
                }));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(()->
                        drive.setPose2D(new Pose2D(DistanceUnit.INCH, 72,72, AngleUnit.RADIANS,Math.toRadians(0)))
                ));

        // ============ Telemetry ============
        register(shooter);
        telemetry.addLine("Init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        loopTimer.resetTimer();

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                drive.getPose().getX(DistanceUnit.INCH),
                drive.getPose().getY(DistanceUnit.INCH),
                drive.getPose().getHeading(AngleUnit.DEGREES)
        );

        telemetry.addData("loop time (ms)",loopTimer.getElapsedTime());
        telemetry.addData("Position", data);
        telemetry.update();
    }
}
