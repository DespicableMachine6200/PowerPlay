package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.ColorSpace;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ml.ModelUnquant;
import org.firstinspires.ftc.teamcode.util.image.TFICBuilder;
import org.firstinspires.ftc.teamcode.util.image.TensorImageClassifier;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.List;

public class ColorDetection extends OpenCvPipeline {
    public boolean done = false;
    public int pos = -1;
    int posX;
    int posY;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public ColorDetection(HardwareMap hardwareMap, Telemetry telemetry, int posX, int posY){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        Bitmap bmp = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input, bmp);
        pos = classifyImage(bmp, input);
        done = true;
        return input;
    }

    public int classifyImage(Bitmap image, Mat imageMat){
        int color = image.getPixel(posX, posY);
        int b = (color)&0xFF;
        int g = (color>>8)&0xFF;
        int r = (color>>16)&0xFF;
        int a = (color>>24)&0xFF;
        int[] colorVals = new int[]{r,g,b};
        String[] colorLabels = new String[]{"A", "B", "C"};
        int max = 0;
        int maxindex = 0;
        for(int i = 0; i < colorVals.length; i++){
            if(colorVals[i] > max){
                max = colorVals[i];
                maxindex = i;
            }
        }
        if(maxindex != 0){
            return labelToId(colorLabels[maxindex]);
        }else{
            return 0;
        }


    }
    int labelToId(String label){
        switch (label){
            case "A":
                return 1;
            case "B":
                return 2;
            case "C":
                return 3;
            default:
                return 0;
        }
    }
}
