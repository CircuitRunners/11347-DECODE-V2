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
import org.firstinspires.ftc.teamcode.commands.CalculateHoodPoseAndVelocity;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;

import java.util.Locale;

@TeleOp
@Config
public class v3MainTeleOp extends CommandOpMode {
    // ========== HARDWARE ==========
    private MecanumDrivebase drive;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;
    private StaticShooter shooter;

    // ========== SOFTWARE ==========
    private boolean isRed = false;
    private final Pose TARGET_GOAL_POSE = new Pose(0, 136);
    private final static double SCORE_ANGLE = Math.toRadians(-30);
    private final static double HOOD_MAX_ANGLE = Math.toRadians(67);
    private final static double HOOD_MIN_ANGLE = Math.toRadians(0);
    private GamepadEx driver;

    @Override
    public void initialize() {
        // ========== HARDWARE ==========
        drive = new MecanumDrivebase(hardwareMap, isRed, false);

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        intake = new IntakeSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap);

        // ========== SOFTWARE ==========
        driver = new GamepadEx(gamepad1);

        // ========== COMMANDS ==========
        drive.setDefaultCommand(new DriveCommand(drive, driver, isRed));
        intake.setDefaultCommand(new IntakeCommand(intake, driver));

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()-> {
                    drive.setPose(new Pose(drive.getPose().getX(), drive.getPose().getY(), Math.toRadians(0.0)));
                }));

        // ========== REGISTER ==========
        register(shooter, hood);
        schedule(new BulkCacheCommand(hardwareMap),
                new PerpetualCommand(
                        new CalculateHoodPoseAndVelocity(
                                drive, shooter, hood, TARGET_GOAL_POSE,
                                0.5, 20.0,
                                SCORE_ANGLE, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE
                        ))
        );

        // ========== SET START POSE ==========
        drive.setStartingPose();

        // ========== TELEMETRY ==========
        telemetry.addLine("Init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                drive.getPose().getX(),
                drive.getPose().getY(),
                drive.getPose().getHeading()
        );

        telemetry.addData("Position", data);
        telemetry.update();
    }
}
