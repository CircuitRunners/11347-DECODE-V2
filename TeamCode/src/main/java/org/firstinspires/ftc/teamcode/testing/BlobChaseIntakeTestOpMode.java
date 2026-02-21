package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.OpenCVPipelines.PurpleGreenBlobPipeline;
import org.openftc.easyopencv.*;

@TeleOp(name = "Blob Chase + Intake Test", group = "Test")
public class BlobChaseIntakeTestOpMode extends OpMode {

    public static String WEBCAM_NAME = "RedWebcam";

    // Centering control
    public static double CENTER_DEADBAND = 0.12;   // cxNorm within +/- => considered centered
    public static double kTurn = 0.35;             // slow turning
    public static double MAX_TURN = 0.25;

    // Slow forward drive while target exists
    public static double FORWARD_WHEN_TARGET = 0.12; // very slow
    public static double FORWARD_WHEN_CENTERED = 0.16; // slightly faster once centered
    public static double MAX_FWD = 0.20;

    // Stop conditions
    public static double X_LIMIT = 130.0;
    public static int BALL_LIMIT = 3;

    private OpenCvCamera camera;
    private PurpleGreenBlobPipeline pipeline;

    @Override
    public void init() {
        pipeline = new PurpleGreenBlobPipeline();

        int camMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, WEBCAM_NAME),
                camMonitorViewId
        );

        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // IMPORTANT: since you're NOT rotating inside the pipeline anymore,
                // keep the stream rotation here as you already have it.
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
            }
        });
    }

    @Override
    public void loop() {
        double poseX = getPoseXInches();  // TODO hook to follower/odometry
        int ballCount = getBallCount();   // TODO hook to your CZD / transfer count

        boolean stopBecauseFull = ballCount >= BALL_LIMIT;
        boolean stopBecauseX = poseX >= X_LIMIT;

        // Intake is ON only while chasing AND not full AND not past X limit
        boolean shouldIntake = !stopBecauseFull && !stopBecauseX && pipeline.hasTarget;
        setIntake(shouldIntake);

        telemetry.addData("poseX", "%.1f", poseX);
        telemetry.addData("ballCount", ballCount);
        telemetry.addData("stopFull", stopBecauseFull);
        telemetry.addData("stopX", stopBecauseX);
        telemetry.addData("hasTarget", pipeline.hasTarget);

        if (!pipeline.hasTarget || stopBecauseFull || stopBecauseX) {
            // Stop movement if no target or stop condition hit
            driveRobot(0.0, 0.0);
            telemetry.addLine(!pipeline.hasTarget ? "No target -> stopped" :
                    stopBecauseFull ? "3 balls -> stopped" : "Past X limit -> stopped");
            telemetry.update();
            return;
        }

        // --- Vision servo: center the blob ---
        double cxNorm = pipeline.cxNorm; // [-1..1], 0 = centered
        boolean centered = Math.abs(cxNorm) <= CENTER_DEADBAND;

        // Turn to reduce cx error (flip TURN_SIGN if backwards)
        double turn = TURN_SIGN * (kTurn * cxNorm);
        turn = clamp(turn, -MAX_TURN, MAX_TURN);

        // Slow forward; slightly more when centered
        double forward = centered ? FORWARD_WHEN_CENTERED : FORWARD_WHEN_TARGET;
        forward = clamp(forward, 0.0, MAX_FWD);

        // Optional: reduce forward while turning harder
        forward *= (1.0 - clamp(Math.abs(turn) / MAX_TURN, 0.0, 1.0) * 0.6);

        driveRobot(forward, turn);

        telemetry.addData("cxNorm", "%.3f", cxNorm);
        telemetry.addData("centered", centered);
        telemetry.addData("cmdFwd", "%.2f", forward);
        telemetry.addData("cmdTurn", "%.2f", turn);
        telemetry.update();
    }

    // Flip if robot turns the wrong direction when blob is to the right/left
    private static final double TURN_SIGN = 1.0;

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // -----------------------
    // TODO: wire these to your robot
    // -----------------------

    private void driveRobot(double forward, double turn) {
        // Replace with your drivebase:
        // - mecanum: forward, strafe=0, turn
        // Example:
        // drive.set(forward, 0.0, turn);
    }

    private void setIntake(boolean on) {
        // Replace with your intake subsystem
        // Example:
        // intake.setPower(on ? 1.0 : 0.0);
    }

    private double getPoseXInches() {
        // Replace with your Pedro follower pose.x or your drive.pose.x
        // Example:
        // return follower.getPose().getX();
        return 0.0;
    }

    private int getBallCount() {
        // Replace with your CZD/tracker ball count
        return 0;
    }

    @Override
    public void stop() {
        if (camera != null) camera.stopStreaming();
        setIntake(false);
        driveRobot(0.0, 0.0);
    }
}
