package org.firstinspires.ftc.teamcode.system.visiontest;

//import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;
//import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_POS;
//import static org.firstinspires.ftc.teamcode.system.hardware.Globals.place;

//import androidx.core.math.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class RailAdjustAprilTag extends AprilTagPipeline
{

    public enum Place
    {
        LEFT,
        RIGHT,
        MIDDLE,
        NONE
    }
    public static Place depositPlace = Place.LEFT;

    private final Object object = new Object();
    ArrayList<Integer> tagList = new ArrayList<>();

    //AprilTagLibrary library = getCenterStageTagLibrary(); // apparently the official one is wrong


    //Pose2d pos = new Pose2d(0, 0, 0);

    double target;

    public RailAdjustAprilTag()
    {
        super();
    }
    public RailAdjustAprilTag(Telemetry telemetry)
    {
        super(telemetry);
    }

    @Override
    public void init(Mat frame)
    {
        super.init(frame);
        if (true)
        {
            tagList.add(1);
            tagList.add(2);
            tagList.add(3);
        }
        else
        {
            tagList.add(4);
            tagList.add(5);
            tagList.add(6);
        }
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
        ArrayList<AprilTagDetection> detections = getDetectionsUpdate();

        if (detections != null && detections.size() > 0)
        {
            for (AprilTagDetection detection : detections)
            {
                if (detection.id == 1)
                {
                    int leftX = Integer.MAX_VALUE;
                    int rightX = Integer.MIN_VALUE;
                    int topY = Integer.MIN_VALUE;
                    int bottomY = Integer.MAX_VALUE;

                    for (Point point : detection.corners)
                    {
                        if (point.x < leftX) leftX = (int) point.x;
                        if (point.x > rightX) rightX = (int) point.x;
                        if (point.y > topY) topY = (int) point.y;
                        if (point.y < bottomY) bottomY = (int) point.y;
                    }

                    int tagCenterX = (int) detection.center.x;
                    int tagCenterY = (int) detection.center.y;

                    int tagWidth = rightX - leftX;
                    //int tagHeight = topY - bottomY;

                    double IN_PER_PIXEL = 2.0 / tagWidth; // tag size is in meters so...

                    double middleScreen = 640; //1280 / 2.0;

                    // this assumes that x axis approaches ∞+ from the left, or left is negative lol
                    // just keep in mind that the servo position far right is 0

                    telemetry.addData("Tag x", tagCenterX);
                    double error = (middleScreen - tagCenterX);

                    switch (depositPlace)
                    {
                        case LEFT:
                            error += -tagWidth * .75;
                            break;
                        case RIGHT:
                            error += tagWidth * .75;
                            break;
                        case MIDDLE:
                            error += 0;
                            break;
                    }
                    Imgproc.circle(input, new Point(error + tagCenterX, tagCenterY - 100), 4, new Scalar(0, 0, 255), 4);
                    Imgproc.drawMarker(input, new Point(middleScreen, 360), new Scalar(255, 0, 0), 2, 15);
                    // until this point everything was in pixels


                    error = error * IN_PER_PIXEL;
                    // here we have the error in inches
                    telemetry.addData("X inches", error);
                    // 0.01 ticks = 0.2805 in  trust me lol

                    error = -error / 14.027 * 0.01;
                    // here we have the error in servo ticks

                    target = error + 0.49;
                    //target = MathUtils.clamp(error + 0.49, 0, 1);

                    // istead of doing a piece wise for the conversion i just add the error to the center position




                    //TODO: conversion rate between servo pos and inches



                    break;
                }
            }
        }
        telemetry.addData("Target", target);
        //telemetry.addData("New position y", );
        telemetry.update();
        return input;
    }

    public double getTarget()
    {
        return target;
    }
}
