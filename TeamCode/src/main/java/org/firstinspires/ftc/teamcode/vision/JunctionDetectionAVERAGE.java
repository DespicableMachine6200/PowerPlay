package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class JunctionDetectionAVERAGE extends OpenCvPipeline {
        public double focal = 399.0; //inch
        public double poleWidth = 1; //inch
        public double width = 10;
        public double distance = 0;
        public double totalDistance = 0;
        public double avgDistance = 0;
        public boolean done = false;
        public boolean started = false;
        public boolean middle = false;
        public boolean end = false;
        ArrayList<Double> distanceList;

        public Mat processFrame(Mat src) {
            started = true;
            System.loadLibrary( Core.NATIVE_LIBRARY_NAME );

            // change yellow to black and everything else to white
            Mat hsv = new Mat();
            Mat src2 = new Mat();
            Imgproc.resize(src, src2, new Size(160, 90), 0.25, 0.25, Imgproc.INTER_AREA);
            Imgproc.cvtColor(src2, hsv, Imgproc.COLOR_BGR2HSV); // hsv mat with hsv values
            Scalar strictLowYellow = new Scalar(15,150,100); // lower yellow bound
            Scalar strictHighYellow = new Scalar(255, 255,255); // higher yellow bound
            //Scalar strictLowYellow = new Scalar(20,70,80);
            //Scalar strictHighYellow = new Scalar(32, 255,255);
            Mat mask = new Mat();;

            // change all non-specified color to white
            Core.inRange(hsv, strictLowYellow, strictHighYellow, mask); // all yellow in hsv in mask
            src2.setTo(new Scalar(0, 0, 0), mask); // write back to src with all yellow black

            // change all non-yellow to white
            Mat maskInv = new Mat();
            Core.bitwise_not(mask, maskInv);
            src2.setTo(new Scalar(255,255,255), maskInv);
            middle = true;
            //Converting the source image to binary
            Mat gray = new Mat(src2.rows(), src2.cols(), src2.type());
            Imgproc.cvtColor(src2, gray, Imgproc.COLOR_BGR2GRAY);
            Mat binary = new Mat(src2.rows(), src2.cols(), src2.type(), new Scalar(0));
            Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

            //Finding Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchey = new Mat();
            Imgproc.findContours(binary, contours, hierarchey, Imgproc.RETR_TREE,
                    Imgproc.CHAIN_APPROX_SIMPLE);
            MatOfPoint contour = contours.get(0);
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

            for(int id = maxValIdx; id < maxValIdx+1; id++) {
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(maxValIdx).toArray()));
                width = Math.min(rotatedRect.size.width, rotatedRect.size.height);
            }

            distanceList.add(calculateDistance(width*4));
            if (distanceList.size() > 5) {
                distanceList.remove(0);
            }
            for (int i = 0; i<distanceList.size(); i++) {
                totalDistance += distanceList.get(i);
            }
            avgDistance = totalDistance/5;
            done = true;
            return src2;
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

