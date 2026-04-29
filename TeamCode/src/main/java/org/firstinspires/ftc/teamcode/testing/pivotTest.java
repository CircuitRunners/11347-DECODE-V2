package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakePivot;

@Config
@TeleOp
public class pivotTest extends CommandOpMode {
    private IntakePivot pivot;

    @Override
    public void initialize() {
        pivot = new IntakePivot(hardwareMap);

        register(pivot);
        schedule(new BulkCacheCommand(hardwareMap));
    }

    @Override
    public void run() {
        super.run();
    }
}
