package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class magneticLimitTest extends CommandOpMode {
    private DigitalChannel magneticLimitSwitch;

    @Override
    public void initialize() {
        magneticLimitSwitch = hardwareMap.get(DigitalChannel.class, "magLimit");
        magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("init done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("State: ", magneticLimitSwitch.getState());
        telemetry.update();
    }
}
