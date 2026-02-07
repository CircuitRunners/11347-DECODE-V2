package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AprilTag.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.concurrent.TimeUnit;

import java.util.ArrayList;

public class WebcamAprilTag extends SubsystemBase {
    private OpenCvWebcam camera;   // <-- change type
    private AprilTagDetectionPipeline pipeline;

    private AprilTagDetection tagOfInterest = null;
    private int detectedTagId = -1;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.1016; // 4 inches (101.6 mm)

    public WebcamAprilTag(HardwareMap hardwareMap, String webcam) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        camera = OpenCvCameraFactory.getInstance().createWebcam(  // <-- returns OpenCvWebcam
                hardwareMap.get(WebcamName.class, webcam),
                cameraMonitorViewId
        );

        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);

                ExposureControl exposure = camera.getExposureControl();
                GainControl gain = camera.getGainControl();
                pipeline.setDecimation(2.0f); // try 2–3 while scanning

                exposure.setMode(ExposureControl.Mode.Manual);
                exposure.setExposure(6, TimeUnit.MILLISECONDS); // try 4–10ms
                gain.setGain(30);                               // try 20–80
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    // Call during init loop
    public void detectDuringInit() {
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

        if (!currentDetections.isEmpty()) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 21 || tag.id == 22 || tag.id == 23) { // 21=GPP, 22=PGP, 23=PPG
                    tagOfInterest = tag;
                    detectedTagId = tag.id;
                    tagFound = true;
                    break;
                }
            }

            if (!tagFound && tagOfInterest != null) {
                detectedTagId = tagOfInterest.id;
            }
        }
    }

    public int getDetectionCount() {
        return pipeline.getLatestDetections().size();
    }

    public int getDetectedTag() {
        AlliancePresets.setCurrentCypherId(detectedTagId);
        return detectedTagId;
    }

    public void stopCamera() {
        if (camera != null) camera.closeCameraDeviceAsync(()-> {});
    }

    public AprilTagDetection getTagOfinterest() {
        return tagOfInterest;
    }
}
