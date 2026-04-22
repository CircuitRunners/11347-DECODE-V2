package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers;

@TeleOp
@Config
public class KickerTest extends CommandOpMode {
    private Kickers kickers;
    public static double z1 = 0, z2 = 0, z3 = 0;

    @Override
    public void initialize() {
        kickers = new Kickers(hardwareMap);
        schedule(new BulkCacheCommand(hardwareMap));
    }

    @Override
    public void run() {
        super.run();

        kickers.setZoneOnePos(z1);
        kickers.setZoneTwoPos(z2);
        kickers.setZoneThreePos(z3);
    }
}
