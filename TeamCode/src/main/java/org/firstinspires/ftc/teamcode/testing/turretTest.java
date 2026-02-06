package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
@Config
public class turretTest extends CommandOpMode {
    private Servo turret1, turret2;

    public static double TURRET_POS_DECIMAL = 0.0;
    public static double TURRET_POS_DEG = TURRET_POS_DECIMAL * 360;

    @Override
    public void initialize() {
        turret1 = hardwareMap.get(Servo.class, "s1");
        turret2 = hardwareMap.get(Servo.class, "s2");

        turret1.setPosition(TURRET_POS_DECIMAL);
        turret2.setPosition(TURRET_POS_DECIMAL);
    }

    @Override
    public void run() {
        super.run();

        turret1.setPosition(TURRET_POS_DECIMAL);
        turret2.setPosition(TURRET_POS_DECIMAL);

        telemetry.addData("Pos1", turret1.getPosition()
                + "Pos2", turret2.getPosition());
        telemetry.update();
    }
}
