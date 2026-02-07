package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter;
import org.firstinspires.ftc.teamcode.support.BeamBreakHelper;

@Config
@TeleOp(name = "BeamBreak Tester", group = "Tuning")
public class BeamBreakTester extends LinearOpMode {

    private BeamBreakHelper outtakeBeam;
    private StaticShooter shooter;
    private IntakeSubsystem intake;

    public static double TARGET = 0;
    public static boolean TRANSFER = false;

    @Override
    public void runOpMode() throws InterruptedException {
        outtakeBeam = new BeamBreakHelper(hardwareMap, "outtakeBeamBreak", 0);

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        intake = new IntakeSubsystem(hardwareMap, telemetry);

        telemetry.addLine("BeamBreak Tester Initialized");
        telemetry.addLine("Press PLAY to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            outtakeBeam.update();
            shooter.update();

            shooter.setTargetRPM(TARGET);

            if (outtakeBeam.getBallCount() >= 3) {
                outtakeBeam.resetBallCount();
            }

            telemetry.addLine("=== OUTTAKE BEAM ===");
            telemetry.addData("Raw", outtakeBeam.isBeamBroken() ? "BROKEN" : "CLEAR");
            telemetry.addData("Stable Broken", outtakeBeam.isBeamStable());
            telemetry.addData("Balls Passed", outtakeBeam.getBallCount());
            telemetry.update();
        }
    }
}
