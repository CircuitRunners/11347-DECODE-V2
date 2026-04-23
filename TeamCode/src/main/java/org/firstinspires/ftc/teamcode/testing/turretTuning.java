package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.MotorTurretTracker;

@Config
@TeleOp
public class turretTuning extends CommandOpMode {
    private MotorTurretTracker turret;
    public static int turretTarget = 0;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

        turret = new MotorTurretTracker(hardwareMap, true);

        register(turret);

        telemetry.addLine("turret test ready");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        turret.setTarget(turretTarget);

        telemetry.addData("Target: ", turret.getTarget());
        telemetry.addData("Current Pose: ", turret.getCurrentEncoderPos());
        telemetry.addData("Current (A): ", turret.getCurrent());
        telemetry.addData("At Limit? ", turret.isAtLimit());
        telemetry.update();
    }
}
