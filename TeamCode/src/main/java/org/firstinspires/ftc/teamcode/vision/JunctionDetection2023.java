package org.firstinspires.ftc.teamcode.vision;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;

public class JunctionDetection2023 extends OpenCvPipeline {
    public double focal = 399.0; //inch
    public double poleWidth = 1.05; //inch
    public double distance = 0;
    public boolean done = false;
    public boolean started = false;
    public boolean middle = false;
    public boolean end = false;
    private Point centroid;
    private Point Test;
    private Point Top;
    public int Y;
    public int X;
    public double y;
    public double x;
    private double Angle;
    public static Scalar DISPLAY_COLOR = new Scalar(210, 150, 190);
    //        public Scalar lower = new Scalar(0, 155, 60); //Tests
    public Scalar lower = new Scalar(8, 68, 155); //Actual
    //        public Scalar upper = new Scalar(30, 220, 120); //Tests
    public Scalar upper = new Scalar(45, 255, 255); //Actual


    public Exception debug;

    public ColorSpace colorSpace = ColorSpace.HSV;

    private Mat HSVMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();

    public Telemetry telemetry;

    enum ColorSpace {
        HSV(Imgproc.COLOR_RGB2HSV);

        public int cvtCode = 0;

        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        try {
            started = true;
            Imgproc.cvtColor(input, HSVMat, colorSpace.cvtCode);
            //Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_BGR2GRAY);
            Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV);

            Core.inRange(HSVMat, lower, upper, binaryMat);
            // Remove Noise
            Imgproc.morphologyEx(binaryMat, binaryMat, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(binaryMat, binaryMat, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.GaussianBlur(binaryMat, binaryMat, new Size(5.0, 5.0), 0.00);
            List<MatOfPoint> contours = new ArrayList<>();
            List<MatOfPoint> filteredContours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(binaryMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(binaryMat, contours, -1, DISPLAY_COLOR, 2);

            middle = true;
            double maxWidth = 0;
            for (MatOfPoint contour : contours) {
                Rect rec = Imgproc.boundingRect(contour);
                double width = rec.width;
                if (width > maxWidth) {
                    maxWidth = width;

                }
            }
            end = true;
            calculateDistance(maxWidth);
            done = true;
            return input;

        } catch (Exception e) {
            debug = e;
            boolean error = true;
        }
        return input;
    }

    public double calculateDistance(double width) {
        // method three - similarity 2d implementation
        if (width == 0) {
            return 0;
        }
        distance = focal * poleWidth / width;
        return distance;
    }
}