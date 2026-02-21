package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase;

import java.util.Locale;

@Config
@TeleOp(group="2")
public class TurretServoRangefinder extends CommandOpMode {
    private Servo turret;
    private MecanumDrivebase drive;
    public static double pose = 0.5;
    private double leftMax = 0.0, rightMax = 0.0;
    public static boolean isRed = true;

    @Override
    public void initialize() {
        turret = hardwareMap.get(Servo.class, "turret");
        turret.setPosition(pose);

        drive = new MecanumDrivebase(hardwareMap, isRed, false);

        telemetry.addLine("initDone");
        telemetry.update();
//        drive.setStaringPose2D();
        drive.setStartingPose();
    }

    @Override
    public void run() {
        super.run();

        if (gamepad1.dpad_left) {
            pose -= 0.02;
        } else if (gamepad1.dpad_right) {
            pose += 0.02;
        }

        if (gamepad1.square) {
            leftMax = turret.getPosition();
        } else if (gamepad1.circle) {
            rightMax = turret.getPosition();
        }

        turret.setPosition(pose);

//        String data = String.format(Locale.US,
//                "{X: %.3f, Y: %.3f, H: %.3f}",
//                drive.getPose().getX(DistanceUnit.INCH),
//                drive.getPose().getY(DistanceUnit.INCH),
//                drive.getPose().getHeading(AngleUnit.DEGREES)
//        );

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                drive.getPose().getX(),
                drive.getPose().getY(),
                drive.getPose().getHeading()
        );

        telemetry.addData("Position", data);
        telemetry.addData("Current Turret Pose:", pose);
        telemetry.addData("LeftMax:", leftMax);
        telemetry.addData("RightMax:", rightMax);
        telemetry.update();
    }
}
