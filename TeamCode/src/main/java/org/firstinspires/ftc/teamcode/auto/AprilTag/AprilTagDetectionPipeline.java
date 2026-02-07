// FILE: org/firstinspires/ftc/teamcode/auto/AprilTag/AprilTagDetectionPipeline.java
// Drop-in replacement for your pipeline with:
// 1) ID filtering (only publish/draw 21/22/23)
// 2) Runtime decimation control (already had it)
// Notes:
// - Keep Tag family 36h11 (same as your current code)
// - tagsize is meters (0.1016 for 4")

package org.firstinspires.ftc.teamcode.auto.AprilTag;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    private long nativeApriltagPtr;
    private final Mat grey = new Mat();

    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    private Mat cameraMatrix;

    private final Scalar blue = new Scalar(7,197,235,255);
    private final Scalar red = new Scalar(255,0,0,255);
    private final Scalar green = new Scalar(0,255,0,255);
    private final Scalar white = new Scalar(255,255,255,255);

    private final double fx, fy, cx, cy;

    // meters
    private final double tagsize;
    private final double tagsizeX;
    private final double tagsizeY;

    private float decimation = 2.0f;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    // ID filter
    private volatile Set<Integer> allowedIds = new HashSet<>(Arrays.asList(21, 22, 23));

    public AprilTagDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy) {
        this.tagsize = tagsize;
        this.tagsizeX = tagsize;
        this.tagsizeY = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        constructMatrix();

        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(
                AprilTagDetectorJNI.TagFamily.TAG_36h11.string,
                3,
                3
        );
    }

    /** Optional: change which IDs are allowed at runtime */
    public void setAllowedIds(int... ids) {
        HashSet<Integer> s = new HashSet<>();
        for (int id : ids) s.add(id);
        allowedIds = s;
    }

    @Override
    public void finalize() {
        if (nativeApriltagPtr != 0) {
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run detector (returns ALL IDs in the family)
        ArrayList<AprilTagDetection> raw =
                AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        // Filter to only your tags (21/22/23)
        ArrayList<AprilTagDetection> filtered = new ArrayList<>();
        Set<Integer> allow = allowedIds;
        for (AprilTagDetection d : raw) {
            if (allow.contains(d.id)) filtered.add(d);
        }

        detections = filtered;
        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        // Draw only your tags
        for (AprilTagDetection detection : detections) {
            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
            drawAxisMarker(input, tagsizeY / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
        }

        return input;
    }

    /**
     * Decimation guidance:
     * - Far targets (144"): use 1.0 (max detail)
     * - Mid: 2.0
     * - Close (20"): 3.0+ (more speed)
     */
    public void setDecimation(float decimation) {
        synchronized (decimationSync) {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate() {
        synchronized (detectionsUpdateSync) {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    private void constructMatrix() {
        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, cx);

        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, fy);
        cameraMatrix.put(1, 2, cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
    }

    private void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    private void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2,  tagHeight/2, 0),
                new Point3( tagWidth/2,  tagHeight/2, 0),
                new Point3( tagWidth/2, -tagHeight/2, 0),
                new Point3(-tagWidth/2, -tagHeight/2, 0),
                new Point3(-tagWidth/2,  tagHeight/2, -length),
                new Point3( tagWidth/2,  tagHeight/2, -length),
                new Point3( tagWidth/2, -tagHeight/2, -length),
                new Point3(-tagWidth/2, -tagHeight/2, -length)
        );

        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        for (int i = 0; i < 4; i++) {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i + 4], blue, thickness);
        }

        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    private Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX, double tagsizeY) {
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2,  tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3( tagsizeX/2,  tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3( tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);
        return pose;
    }

    private static class Pose {
        Mat rvec = new Mat();
        Mat tvec = new Mat();
    }
}