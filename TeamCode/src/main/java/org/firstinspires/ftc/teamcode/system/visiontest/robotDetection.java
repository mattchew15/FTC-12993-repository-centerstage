package org.firstinspires.ftc.teamcode.system.visiontest;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.visiontest.AprilTagPipeline;
import org.opencv.core.Mat;


public class robotDetection extends AprilTagPipeline
{
    public static boolean ClearPath;
    public robotDetection(Telemetry telemetry)
    {
        super(telemetry);
        
        ClearPath = true;
    }

    @Override
    public void init(Mat frame)
    {
        super.init(frame);
    }

    @Override
    public void finalize()
    {
        super.finalize();
    }

    @Override
    public Mat processFrame(Mat input)
    {
        super.processFrame(input);
        short count = getDetectionsCounter();
        ClearPath = count >= 2;
        telemetry.addData("Count", count);
        telemetry.addData("ClearPath", ClearPath);
        return input;
    }
}
