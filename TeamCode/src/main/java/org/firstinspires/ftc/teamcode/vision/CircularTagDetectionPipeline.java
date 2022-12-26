package org.firstinspires.ftc.teamcode.vision;

import android.content.Context;
import android.graphics.Bitmap;

import org.firstinspires.ftc.teamcode.ml.ModelUnquant;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.android.Utils;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class CircularTagDetectionPipeline extends OpenCvPipeline {
    public boolean done = false;
    public int pos = -1;
    @Override
    public Mat processFrame(Mat input) {
        Bitmap bmp = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input, bmp);
        pos = classifyImage(bmp);
        done = true;
        return input;
    }

    public int classifyImage(Bitmap image){
        try {
            ModelUnquant model = ModelUnquant.newInstance(null);
            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();

            TensorBuffer inputFeature0 = TensorBuffer.createFixedSize(new int[]{1, 224, 224, 3}, DataType.FLOAT32);
            ByteBuffer byteBuffer = ByteBuffer.allocateDirect(4*imageWidth*imageHeight*3);
            byteBuffer.order(ByteOrder.nativeOrder());
            int[] intValues = new int[imageWidth*imageHeight];
            image.getPixels(intValues, 0, image.getWidth(),0, 0, image.getWidth(), image.getHeight());
            int pixel = 0;
            for(int i = 0; i < imageWidth; i++){
                for(int j = 0; j < imageHeight; j++){
                    int val = intValues[pixel++];
                    //bitwise operations yay i definitely don't hate these!
                    byteBuffer.putFloat(((val >> 16)&0xFF)*(1.f/255.f));
                    byteBuffer.putFloat(((val >> 8)&0xFF)*(1.f/255.f));
                    byteBuffer.putFloat((val & 0xFF)*(1.f/255.f));
                }
            }

            inputFeature0.loadBuffer(byteBuffer);


            ModelUnquant.Outputs outputs = model.process(inputFeature0);
            TensorBuffer outputFeature0 = outputs.getOutputFeature0AsTensorBuffer();
            float[] confidences = outputFeature0.getFloatArray();
            int maxPos = 0;
            float maxConfidence = 0;
            for(int i = 0; i < confidences.length; i++){
                if(confidences[i] > maxConfidence){
                    maxConfidence = confidences[i];
                    maxPos = i;
                }
            }
            String[] classes = {"1", "2", "3"};
            int posReturn = Integer.parseInt(classes[maxPos]);
            model.close();
            return posReturn;
        } catch (IOException e) {
            e.printStackTrace();;
            return 0;
        }
    }
}
