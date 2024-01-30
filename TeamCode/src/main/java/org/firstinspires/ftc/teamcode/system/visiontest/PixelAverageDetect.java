package org.firstinspires.ftc.teamcode.system.visiontest;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

public class PixelAverageDetect extends OpenCvPipeline
{
    public Scalar lowerBound = new Scalar(73);
    public Scalar upperBound = new Scalar(255);
    private short pos;

    public Telemetry telemetry;

    // In init it should figure out the camera dimensions
    private  short
            leftXPoint = 0,
            middleXPoint = 426,
            rightXPoint = 852,
            leftYPoint = 0,
            middleYPoint = 0,
            rightYPoint = 0;

    private short
    regionWidth = 426,
    regionHeight = 800;
    private Point
            regionLeftA = new Point(leftXPoint, leftYPoint),
            regionLeftB = new Point(leftXPoint + regionWidth, leftYPoint + regionHeight),
            regionMiddleA = new Point(middleXPoint, middleYPoint),
            regionMiddleB = new Point(middleXPoint + regionWidth, middleYPoint + regionHeight),
            regionRightA = new Point(rightXPoint, rightYPoint),
            regionRightB = new Point(rightXPoint + regionWidth, rightYPoint + regionHeight);

    private Mat regionLeftLight, regionMiddleLight, regionRightLight;
    private Mat grayImage = new Mat(), channelMat = new Mat();

    public PixelAverageDetect(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    public void inputToGrayScale(Mat input)
    {
        Imgproc.cvtColor(input, grayImage, Imgproc.COLOR_RGB2GRAY);
        Core.extractChannel(grayImage, channelMat, 0);
    }
    @Override
    public void init(Mat frame)
    {

        inputToGrayScale(frame);

        regionLeftLight = channelMat.submat(new Rect(regionLeftA, regionLeftB));
        regionMiddleLight = channelMat.submat(new Rect(regionMiddleA, regionMiddleB));
        regionRightLight = channelMat.submat(new Rect(regionRightA, regionRightB));
    }
    @Override
    public Mat processFrame(Mat input)
    {
        inputToGrayScale(input);

        //regionLeftLight = grayImage.submat(new Rect(regionLeftA, regionLeftB));
        //regionMiddleLight = grayImage.submat(new Rect(regionMiddleA, regionMiddleB));
        //regionRightLight = grayImage.submat(new Rect(regionRightA, regionRightB));

        short avgLeft = (short) Core.mean(regionLeftLight).val[0];
        short avgMiddle = (short) Core.mean(regionMiddleLight).val[0];
        short avgRight = (short) Core.mean(regionRightLight).val[0];

        Imgproc.rectangle(input, regionLeftA, regionLeftB, new Scalar(0,0,255), 2);
        Imgproc.rectangle(input, regionMiddleA, regionMiddleB, new Scalar(0,0,255), 2);
        Imgproc.rectangle(input, regionRightA, regionRightB, new Scalar(0,0,255), 2);

        short max = (short) Math.max(avgLeft, Math.max(avgMiddle, avgRight));
        if (max == avgLeft)
        {
            pos = 0;
        }
        else if (max == avgRight)
        {
            pos = 2;
        }
        else
        {
            pos = 1;
        }

        telemetry.addData("Pos", pos);
        telemetry.update();

        //grayImage.release();
        //channelMat.release();
        return input;
    }
    @Override
    public void finalize()
    {
        //grayImage.release();
        //channelMat.release();
    }
    public short getPos()
    {
        return pos;
    }
    public void setDimensions(short width, short height)
    {
        regionWidth = (short) (width / 3);
        regionHeight = height;

        leftXPoint = 0;
        middleXPoint = (short) (leftXPoint + regionWidth);
        rightXPoint = (short) (middleXPoint + regionWidth);

        regionLeftA = new Point(leftXPoint, leftYPoint);
        regionLeftB = new Point(leftXPoint + regionWidth, leftYPoint + regionHeight);
        regionMiddleA = new Point(middleXPoint, middleYPoint);
        regionMiddleB = new Point(middleXPoint + regionWidth, middleYPoint + regionHeight);
        regionRightA = new Point(rightXPoint, rightYPoint);
        regionRightB = new Point(rightXPoint + regionWidth, rightYPoint + regionHeight);

    }
}
