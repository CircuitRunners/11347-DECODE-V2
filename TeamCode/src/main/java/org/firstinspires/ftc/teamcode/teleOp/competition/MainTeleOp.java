package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;


import java.util.Locale;

@Config
@TeleOp(group="1")
public class MainTeleOp extends CommandOpMode{
    private StaticShooter shooter;
    private Turret turret;
    private GamepadEx driver, manipulator;
    private GoBildaPinpointDriver pinpoint;
    private MecanumDrivebase drive;
    private Kickers Kickers;
    private double turretPosition;
    private int servoNumber = 0;
    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        turret = new Turret(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        //Kickers = new Kickers(hardwareMap);

        drive = new MecanumDrivebase(hardwareMap);

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(()-> turretPosition = turretPosition+0.01));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(()-> turretPosition = turretPosition-0.01));

        //transfer commands
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(()-> {
                    servoNumber = (servoNumber+1) % 3;

                }))
                .whenReleased(new InstantCommand(()-> {


                }));


        //shooter stuff
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(()-> {
                    shooter.setTargetRPM(3500);
//                    out.aimScoring();
                }));

        // Click bumper once to activate intake at close speed
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(()-> {
                    shooter.setTargetRPM(2500);
//                    out.aimClose();
                }));
        
        Trigger shooterOff = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER));
        shooterOff.whenActive(new InstantCommand(() -> shooter.setTargetRPM(0)));
    }


    @Override
    public void run() {
        super.run();

        turret.setTurretPosition(turretPosition);


        double forward = driver.getLeftY(); // Forwards/backwards
        double right = driver.getLeftX(); // Strafe
        double rotate = driver.getRightX(); // Rotation

        // --- Telemetry ---

        Pose2D pose;
        pose = driveFieldRelative(forward, right, rotate);
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES)
        );

        telemetry.addLine("----  Subsystems Data  ----");
        telemetry.addData("Aiming Turret Pos: ", turret.getTurretPosition());
        telemetry.addLine("----  Pinpoint Data  ----");
        telemetry.addData("Position", data);
        telemetry.update();
    }
    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();

        Pose2D pos = pinpoint.getPosition();  // Current position
//
        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
                .normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
        return pos;
    }

    private void configurePinpoint() {
        pinpoint.resetPosAndIMU();

        pinpoint.setOffsets(147.225, 142.225, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
    }
}
