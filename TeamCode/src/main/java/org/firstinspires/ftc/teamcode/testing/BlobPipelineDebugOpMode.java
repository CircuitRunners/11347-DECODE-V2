package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.OpenCVPipelines.PurpleGreenBlobPipeline;
import org.openftc.easyopencv.*;

@TeleOp(name = "Blob Pipeline Debug", group = "Test")
public class BlobPipelineDebugOpMode extends OpMode {

    public enum Alliance { RED, BLUE }
    public static Alliance ALLIANCE = Alliance.RED; // change as needed

    // Which webcam config name to use
    public static String WEBCAM_NAME_RED  = "Webcam 1"; // set to your C920 config name
    public static String WEBCAM_NAME_BLUE = "Webcam 2"; // set to your C270 config name

    // Classification tuning
    public static double CENTER_DEADBAND = 0.15;  // cxNorm in [-0.15..+0.15] => CENTER
    public static double RED_TARGET_CX01 = 0.85;  // for red, you want blob to the RIGHT side
    public static double BLUE_TARGET_CX01 = 0.50; // for blue, you want blob centered
    public static double TARGET_TOL_CX01 = 0.06;  // alignment tolerance in [0..1] units

    // Camera stream settings
    public static int STREAM_W = 640;
    public static int STREAM_H = 480;

    private OpenCvCamera camera;
    private PurpleGreenBlobPipeline pipeline;

    @Override
    public void init() {
        pipeline = new PurpleGreenBlobPipeline();

        // If “left/right” is flipped on-screen, flip this
        PurpleGreenBlobPipeline.ROTATE_CW = true;

        int camMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        String webcamName = (ALLIANCE == Alliance.RED) ? WEBCAM_NAME_RED : WEBCAM_NAME_BLUE;

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamName),
                camMonitorViewId
        );

        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(STREAM_W, STREAM_H, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
            }
        });
    }

    @Override
    public void loop() {
        telemetry.addData("Alliance", ALLIANCE);

        if (!pipeline.hasTarget) {
            telemetry.addLine("Target: NONE");
            telemetry.update();
            return;
        }

        // Raw outputs
        double cxNorm = pipeline.cxNorm; // [-1..1]
        double cyNorm = pipeline.cyNorm; // [0..1]
        double area   = pipeline.area;

        telemetry.addLine("Target: YES");
        telemetry.addData("cxNorm [-1..1]", "%.3f", cxNorm);
        telemetry.addData("cyNorm [0..1]", "%.3f", cyNorm);
        telemetry.addData("area", "%.0f", area);

        // Simple left/center/right classification around center of image
        String lr;
        if (cxNorm < -CENTER_DEADBAND) lr = "LEFT";
        else if (cxNorm > CENTER_DEADBAND) lr = "RIGHT";
        else lr = "CENTER";
        telemetry.addData("Blob Position", lr);

        // Alliance-specific “intake alignment” target:
        // convert cxNorm [-1..1] -> cx01 [0..1]
        double cx01 = (cxNorm + 1.0) * 0.5;

        double targetCx01 = (ALLIANCE == Alliance.RED) ? RED_TARGET_CX01 : BLUE_TARGET_CX01;
        double err = targetCx01 - cx01;

        boolean aligned = Math.abs(err) <= TARGET_TOL_CX01;

        telemetry.addData("Target cx01", "%.2f", targetCx01);
        telemetry.addData("Current cx01", "%.2f", cx01);
        telemetry.addData("Err (target-current)", "%.2f", err);
        telemetry.addData("Aligned?", aligned);

        // Optional “guidance” text
        // If err>0, blob is left of where you want it; if err<0, blob is right of target.
        String guidance = (err > TARGET_TOL_CX01) ? "Need blob MORE RIGHT (turn/move accordingly)"
                : (err < -TARGET_TOL_CX01) ? "Need blob MORE LEFT (turn/move accordingly)"
                : "On target";
        telemetry.addData("Guidance", guidance);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (camera != null) camera.stopStreaming();
    }
}
