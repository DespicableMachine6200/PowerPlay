package org.firstinspires.ftc.teamcode.vision;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Size;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class JunctionDetectionPipelineRevised extends OpenCvPipeline {
    public double focal = 399.0; //inch
    public double poleWidth = 1.05; //inch
    public double distance = 0;
    public boolean done = false;
    public boolean started = false;
    public boolean middle = false;
    public boolean end = false;

    public Mat processFrame(Mat src) {
                // read file and stuff
                started = true;
                System.loadLibrary(Core.NATIVE_LIBRARY_NAME );
                Mat src2 = new Mat();
                Mat mask = new Mat();
                Mat hierarchy = new Mat();
                Imgproc.resize(src, src2, new Size(160, 90), 0.25, 0.25, Imgproc.INTER_AREA);

                // change yellow to black and everything else to white
                Imgproc.cvtColor(src2, src2, Imgproc.COLOR_BGR2HSV); // hsv mat with hsv values
                Scalar strictLowYellow = new Scalar(110,100,100); // lower yellow bound
                Scalar strictHighYellow = new Scalar(160, 255,255); // higher yellow bound

                // change all non-specified color to white
                Core.inRange(src2, strictLowYellow, strictHighYellow, mask); // all yellow in hsv in mask
                src2.setTo(new Scalar(0, 0, 0), mask); // write back to src with all yellow black

                middle = true;
                //Finding Contours
                List<MatOfPoint> contours = new ArrayList<>();
                Imgproc.findContours(src2, contours, hierarchy, Imgproc.RETR_TREE,
                        Imgproc.CHAIN_APPROX_SIMPLE);

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
                end = true;
                // draw rectangle for max area contour
                //Scalar green = new Scalar(81, 190, 0);
                double detectedWidth = 0;
                for(int id = maxValIdx; id < maxValIdx+1; id++) {
                    RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(maxValIdx).toArray()));
                    detectedWidth = Math.min(rotatedRect.size.width, rotatedRect.size.height);
                }
                calculateDistance(detectedWidth);
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
