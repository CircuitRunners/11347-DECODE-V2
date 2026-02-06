package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class TurretServoRangefinder extends CommandOpMode {
    private Servo turret;
    public static double pose = 0.5;
    private double leftMax = 0.0, rightMax = 0.0;

    @Override
    public void initialize() {
        turret = hardwareMap.get(Servo.class, "turret");
        turret.setPosition(pose);

        telemetry.addLine("initDone");
        telemetry.update();
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

        telemetry.addData("Current Pose:", pose);
        telemetry.addData("LeftMax:", leftMax);
        telemetry.addData("RightMax:", rightMax);
        telemetry.update();
    }
}
