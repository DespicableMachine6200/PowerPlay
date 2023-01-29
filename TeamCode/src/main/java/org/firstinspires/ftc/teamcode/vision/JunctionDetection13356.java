package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

// @Config - dashboard tuning
public class JunctionDetection13356 extends OpenCvPipeline{
        //backlog of frames to average out to reduce noise
        ArrayList<double[]> frameList;
        //these are public static to be tuned in dashboard
        public static double strictLowS = 140;
        public static double strictHighS = 255;
        public double distance = 0;
        public double focal = 399.0; //inch
        public double poleWidth = 1.05; //inch
        public boolean start = false;
        public boolean middle = false;
        public boolean end = false;
        public boolean done = false;

        public void StickObserverPipeline() {
            frameList = new ArrayList<>();
        }

        @Override
        public Mat processFrame(Mat input) {
            start = true;
            Mat mat = new Mat();

            //mat turns into HSV value
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            if (mat.empty()) {
                return input;
            }

            // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
            Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
            Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

            Mat thresh = new Mat();

            // Get a black and white image of yellow objects
            Core.inRange(mat, lowHSV, highHSV, thresh);

            Mat masked = new Mat();
            //color the white portion of thresh in with HSV from mat
            //output into masked
            Core.bitwise_and(mat, mat, masked, thresh);
            //calculate average HSV values of the white thresh values
            Scalar average = Core.mean(masked, thresh);

            Mat scaledMask = new Mat();
            //scale the average saturation to 150
            masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


            Mat scaledThresh = new Mat();
            //you probably want to tune this
            Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
            Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
            //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
            Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

            Mat finalMask = new Mat();
            //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
            Core.bitwise_and(mat, mat, finalMask, scaledThresh);

            Mat edges = new Mat();
            //detect edges(only useful for showing result)(you can delete)
            Imgproc.Canny(scaledThresh, edges, 100, 200);
            middle = true;
            //contours, apply post processing to information
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            //find contours, input scaledThresh because it has hard edges
            Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
            if (frameList.size() > 5) {
                frameList.remove(0);
            }
            end = true;
            // find the contour w/ max area
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

            // draw rectangle for max area contour
            //Scalar green = new Scalar(81, 190, 0);
            double detectedWidth = 0;
            for(int id = maxValIdx; id < maxValIdx+1; id++) {
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(maxValIdx).toArray()));
                detectedWidth = Math.min(rotatedRect.size.width, rotatedRect.size.height);
            }
            calculateDistance(detectedWidth);

            //release all the data
            scaledThresh.copyTo(input);
            scaledThresh.release();
            scaledMask.release();
            mat.release();
            masked.release();
            edges.release();
            thresh.release();
            finalMask.release();
            done = true;
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
