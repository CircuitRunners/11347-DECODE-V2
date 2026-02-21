package org.firstinspires.ftc.teamcode.auto.OpenCVPipelines;

import org.opencv.core.*;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PurpleGreenBlobPipeline extends OpenCvPipeline {
    public static boolean ROTATE_CW = true; // (unused now, kept so nothing else breaks)

    public static double MIN_AREA = 800;
    public static double MIN_GREEN_FRAC = 0.08;
    public static double MIN_PURPLE_FRAC = 0.08;

    // Tune per venue/camera
    public static Scalar GREEN_LO = new Scalar(35, 60, 30);
    //public static Scalar GREEN_LO = new Scalar(35, 80, 60);
    public static Scalar GREEN_HI = new Scalar(95, 255, 255);
    public static Scalar PURPLE_LO = new Scalar(120, 50, 25);
    //public static Scalar PURPLE_LO = new Scalar(120, 60, 40);
    public static Scalar PURPLE_HI = new Scalar(170, 255, 255);

    // Optional second purple band if needed (set lo=hi to disable)
    public static Scalar PURPLE2_LO = new Scalar(0, 0, 0);
    public static Scalar PURPLE2_HI = new Scalar(0, 0, 0);

    public volatile boolean hasTarget = false;
    public volatile double cxNorm = 0.0; // [-1..1]
    public volatile double cyNorm = 0.0; // [0..1]
    public volatile double area = 0.0;

    private final Mat rgb = new Mat();
    private final Mat hsv = new Mat();

    private final Mat greenMask = new Mat();
    private final Mat purpleMask1 = new Mat();
    private final Mat purpleMask2 = new Mat();
    private final Mat purpleMask = new Mat();

    private final Mat unionMask = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    private final CLAHE clahe = Imgproc.createCLAHE(2.0, new Size(8, 8));

    @Override
    public Mat processFrame(Mat input) {

        // Convert to RGB then HSV (EasyOpenCV often provides RGBA)
        Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);

        // CLAHE on V channel (HSV -> split -> enhance V -> merge)
        List<Mat> channels = new ArrayList<>(3);
        Core.split(hsv, channels);

        clahe.apply(channels.get(2), channels.get(2));  // V channel

        Core.merge(channels, hsv);

        for (Mat m : channels) m.release();

        // Masks (ONLY ONCE)
        Core.inRange(hsv, GREEN_LO, GREEN_HI, greenMask);
        Core.inRange(hsv, PURPLE_LO, PURPLE_HI, purpleMask1);

        boolean usePurple2 = !isDisabledRange(PURPLE2_LO, PURPLE2_HI);
        if (usePurple2) {
            Core.inRange(hsv, PURPLE2_LO, PURPLE2_HI, purpleMask2);
            Core.bitwise_or(purpleMask1, purpleMask2, purpleMask);
        } else {
            purpleMask1.copyTo(purpleMask);
        }

        Core.bitwise_or(greenMask, purpleMask, unionMask);

        // Clean up
        Imgproc.morphologyEx(unionMask, unionMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(unionMask, unionMask, Imgproc.MORPH_CLOSE, kernel);

        // Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(unionMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double bestScore = -1;
        Rect bestRect = null;

        for (MatOfPoint c : contours) {
            double a = Imgproc.contourArea(c);
            if (a < MIN_AREA) continue;

            Rect r = Imgproc.boundingRect(c);

            Mat greenROI = greenMask.submat(r);
            Mat purpleROI = purpleMask.submat(r);

            double denom = (double) (r.width * r.height);
            double greenFrac = Core.countNonZero(greenROI) / denom;
            double purpleFrac = Core.countNonZero(purpleROI) / denom;

            greenROI.release();
            purpleROI.release();

            if (greenFrac < MIN_GREEN_FRAC || purpleFrac < MIN_PURPLE_FRAC) continue;

            if (a > bestScore) {
                bestScore = a;
                bestRect = r;
            }
        }

        if (bestRect != null) {
            hasTarget = true;
            area = bestScore;

            double cx = bestRect.x + bestRect.width / 2.0;
            double cy = bestRect.y + bestRect.height / 2.0;

            int W = input.cols();
            int H = input.rows();

            cxNorm = (cx - W / 2.0) / (W / 2.0);
            cyNorm = (cy / (double) H);

            Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cx, cy), 4, new Scalar(255, 0, 0), -1);
        } else {
            hasTarget = false;
            area = 0;
            cxNorm = 0;
            cyNorm = 0;
        }

        return input;
    }

    private boolean isDisabledRange(Scalar lo, Scalar hi) {
        return lo.val[0] == 0 && lo.val[1] == 0 && lo.val[2] == 0
                && hi.val[0] == 0 && hi.val[1] == 0 && hi.val[2] == 0;
    }
}