package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AprilTag.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="AprilTag Webcam 21/22/23 (Dynamic)", group="Test")
public class AprilTagWebcamOpMode extends LinearOpMode {
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline pipeline;

    private final double fx = 578.272;
    private final double fy = 578.272;
    private final double cx = 402.145;
    private final double cy = 221.506;

    // 4" tag in meters
    private final double tagSizeM = 0.1016;

    // Decimation thresholds (meters)
    private final double CLOSE_Z_M = 0.75;   // ~29.5"
    private final double FAR_Z_M   = 2.0;    // ~79" (anything beyond this is "far-ish")

    @Override
    public void runOpMode() {
        int viewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                viewId
        );

        pipeline = new AprilTagDetectionPipeline(tagSizeM, fx, fy, cx, cy);
        pipeline.setAllowedIds(21, 22, 23);
        pipeline.setDecimation(1.0f); // start in "far detection" mode

        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                // For 144", resolution matters. If your phone bandwidth/RC can handle it, go higher:
                // 1280x720 is best for long range; 800x448 is OK but shorter range.
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override public void onError(int errorCode) {}
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        ElapsedTime dt = new ElapsedTime();
        int bestId = -1;

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> dets = pipeline.getLatestDetections();

            if (dets == null || dets.isEmpty()) {
                // No tag seen: maximize detail for long-range (144")
                pipeline.setDecimation(1.0f);
                bestId = -1;

                telemetry.addLine("No tags (21/22/23) detected");
                telemetry.addData("Decimation", "1.0 (FAR SEARCH)");
            } else {
                // Pick the largest tag in view (usually the closest/most reliable)
                AprilTagDetection best = dets.get(0);
                for (AprilTagDetection d : dets) {
                    if (d.decisionMargin > best.decisionMargin) best = d;
                }
                bestId = best.id;

                // best.pose.z is meters from camera to tag (forward)
                double z = best.pose.z;

                // Dynamic decimation:
                // - close: go faster
                // - mid: balanced
                // - far: keep detail
                if (z <= CLOSE_Z_M) {
                    pipeline.setDecimation(3.0f);
                    telemetry.addData("Decimation", "3.0 (CLOSE SPEED)");
                } else if (z >= FAR_Z_M) {
                    pipeline.setDecimation(1.0f);
                    telemetry.addData("Decimation", "1.0 (FAR DETAIL)");
                } else {
                    pipeline.setDecimation(2.0f);
                    telemetry.addData("Decimation", "2.0 (MID)");
                }

                telemetry.addData("Tag ID", bestId);
                telemetry.addData("Decision Margin", "%.2f", best.decisionMargin);
                telemetry.addData("Pose z (m)", "%.3f", z);
                telemetry.addData("Pose z (in)", "%.1f", z * 39.3701);
                telemetry.addData("Pose x (in)", best.pose.x * 39.3701);
                telemetry.addData("Pose y (in)", best.pose.y * 39.3701);
            }

            telemetry.addData("Loop ms", "%.1f", dt.milliseconds());
            dt.reset();
            telemetry.update();

            sleep(10);
        }

        camera.stopStreaming();
        camera.closeCameraDeviceAsync(() -> {});
    }
}