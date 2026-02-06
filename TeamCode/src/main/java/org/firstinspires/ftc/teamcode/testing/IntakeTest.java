package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

@Config
@TeleOp
public class IntakeTest extends CommandOpMode {
    private IntakeSubsystem intake;
    public static double chubIntakePower, ehubIntakePower;

    public void initialize() {
        intake = new IntakeSubsystem(hardwareMap, telemetry);

        telemetry.update();
    }

    public void run() {
        super.run();

        intake.intakeChub(chubIntakePower);
        intake.intakeEhub(ehubIntakePower);
    }
}
