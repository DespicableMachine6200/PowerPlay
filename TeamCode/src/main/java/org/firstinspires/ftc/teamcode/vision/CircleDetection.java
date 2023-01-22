package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;

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

public class CircleDetection extends OpenCvPipeline {
    public boolean done = false;
    public int pos = -1;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public CircleDetection(HardwareMap hardwareMap, Telemetry telemetry){
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
        imageMat.reshape(224, 224);
        TensorImageClassifier tfic;
        try {
            tfic = new TFICBuilder(hardwareMap, "model.tflite", "A", "B", "C").build();
        } catch (IOException e) {
            return 0;
        }
        List<TensorImageClassifier.Recognition> recognitions = tfic.recognize(imageMat);
        float maxConfidence = 0;
        String maxLabel = null;
        for(int i = 0; i < recognitions.size(); i++){
            TensorImageClassifier.Recognition recognition = recognitions.get(i);
            if(recognition.getConfidence() > maxConfidence){
                maxConfidence = recognition.getConfidence();
                maxLabel = recognition.getTitle();
            }

        }
        return labelToId(maxLabel);
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