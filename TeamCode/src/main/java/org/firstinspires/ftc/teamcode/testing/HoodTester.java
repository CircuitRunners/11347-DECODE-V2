package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class HoodTester extends CommandOpMode {
    private Servo hoodServo;
    public static double position = 0.5;

    @Override
    public void initialize() {
        hoodServo = hardwareMap.get(Servo.class, "hood");

        hoodServo.setPosition(0.5);

        telemetry.addLine("init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        hoodServo.setPosition(position);

        telemetry.addData("hood Pose:", position);
        telemetry.update();
    }
}
