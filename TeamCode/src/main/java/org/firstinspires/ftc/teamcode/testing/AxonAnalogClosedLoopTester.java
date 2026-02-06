package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.shooter.AxonAnalogClosedLoop;

@Disabled
@Deprecated
@TeleOp(name = "Test: Axon Analog Closed Loop", group = "Test")
@Config
public class AxonAnalogClosedLoopTester extends OpMode {

    public static String SERVO_NAME = "axon";
    public static String ANALOG_NAME = "axonEnc";

    public static double GEAR_RATIO = 1.0;
    public static double OFFSET_DEG = 0.0;

    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;

    public static double TARGET_DEG = 0.0;
    public static boolean INVERT_OUTPUT = false;

    public static boolean STOP = false;

    private AxonAnalogClosedLoop controller;
    private MultipleTelemetry tele;

    @Override
    public void init() {
        // This line is what ensures output goes to BOTH Driver Station telemetry and FTC Dashboard telemetry.
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new AxonAnalogClosedLoop(hardwareMap, SERVO_NAME, ANALOG_NAME);

        // (Optional but recommended) ensure dashboard is enabled
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);

        tele.addLine("Initialized. Open FTC Dashboard > Telemetry to view data.");
        tele.update();
    }

    @Override
    public void loop() {
        controller.setGearRatio(GEAR_RATIO);
        controller.setOffsetRad(Math.toRadians(OFFSET_DEG));
        controller.setPID(KP, KI, KD);

        double posDeg = Math.toDegrees(controller.getMechPosRad());

        double targetDeg = TARGET_DEG;
        if (INVERT_OUTPUT) {
            targetDeg = (2.0 * posDeg) - TARGET_DEG; // mirror around current pos
        }

        double errDeg = targetDeg - posDeg;

        if (STOP) {
            controller.stop();
        } else {
            controller.setTargetMechRad(Math.toRadians(targetDeg));
            controller.update();
        }

        // Dashboard + DS telemetry (because we're using "tele")
        tele.addData("posDeg", "%.2f", posDeg);
        tele.addData("targetDeg", "%.2f", TARGET_DEG);
        tele.addData("effectiveTargetDeg", "%.2f", targetDeg);
        tele.addData("errDeg", "%.2f", errDeg);
        tele.addData("KP/KI/KD", "%.5f / %.5f / %.5f", KP, KI, KD);
        tele.addData("gearRatio", "%.4f", GEAR_RATIO);
        tele.addData("offsetDeg", "%.2f", OFFSET_DEG);
        tele.addData("invertOutput", INVERT_OUTPUT);
        tele.addData("STOP", STOP);
        tele.update();
    }

    @Override
    public void stop() {
        if (controller != null) controller.stop();
    }
}
