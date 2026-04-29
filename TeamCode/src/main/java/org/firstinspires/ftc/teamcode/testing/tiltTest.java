package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.tiltClimb;

@Config
@TeleOp
public class tiltTest extends CommandOpMode {
    private static tiltClimb climb;
    public static boolean execute = false;

    private boolean lastExecute = false;

    @Override
    public void initialize() {
        climb = new tiltClimb(hardwareMap);

        register(climb);
        schedule(new BulkCacheCommand(hardwareMap));
    }

    @Override
    public void run() {
        super.run();

        if (execute && !lastExecute) {
            climb.startTiltSequence();
        }

        lastExecute = execute;
    }
}