package org.firstinspires.ftc.teamcode.auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class JunctionDetection extends OpenCvPipeline {

    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 50;
    }

    @Override
    public Mat processFrame(Mat input){
        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if(mat.empty()){
            return input;
        }
        Scalar lowYellow = new Scalar(20,70,80);
        Scalar highYellow = new Scalar(32, 255,255);
        Mat thresh = new Mat();

        Core.inRange(mat, lowYellow, highYellow, thresh);

        Mat masked = new Mat();

        Core.bitwise_and(mat, mat, masked, thresh);

        Scalar average = Core.mean(masked, thresh);
        Mat scaledMask = new Mat();
        masked.convertTo(scaledMask, -1, 150/average.val[1], 0);

        Scalar strictLowYellow = new Scalar(0,150,100);
        Scalar strictHighYellow = new Scalar(255, 255,255);
        Mat scaledThresh = new Mat();
        Core.inRange(scaledMask, strictLowYellow, strictHighYellow, scaledThresh);

        Mat finalMask = new Mat();
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);
        Mat edges = new Mat();
        Imgproc. Canny(finalMask, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat Yellowcontours = new Mat();
        Imgproc.findContours(scaledThresh, contours, Yellowcontours, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);


        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        Moments YellowMoments = Imgproc.moments(contours.get(maxValIdx));
        double cx = 0;
        double cy = 0;
        if (YellowMoments.m00 != 0) {
            cx = YellowMoments.m10 / YellowMoments.m00;
            cy = YellowMoments.m01 / YellowMoments.m00;
        }
        Imgproc.drawContours(Yellowcontours, contours, maxValIdx, new Scalar(0,255,0), 5);

        contours.clear();

        input.release();
        mat.release();
        thresh.release();
        masked.release();
        scaledMask.release();
        scaledThresh.release();
        finalMask.release();
        //edges.release();
        edges.copyTo(input);
        // Imgproc.cvtColor(edges, input, Imgproc.COLOR_HSV2RGB);

        return input;
    }
}