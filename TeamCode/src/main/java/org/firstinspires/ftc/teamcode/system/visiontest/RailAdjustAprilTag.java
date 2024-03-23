package org.firstinspires.ftc.teamcode.system.visiontest;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.place;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class RailAdjustAprilTag extends AprilTagPipeline
{

    Globals.Place depositPlace = Globals.Place.MIDDLE;

    private final Object object = new Object();
    ArrayList<Integer> tagList = new ArrayList<>();

    AprilTagLibrary library = getCenterStageTagLibrary(); // apparently the official one is wrong

    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();

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
        if (BLUE_AUTO)
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
                if (tagList.contains(detection.id))
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

                    double IN_PER_PIXEL = (TAG_SIZE / tagWidth) * 39.3701; // tag size is in meters so...

                    double middleScreen = 640; //1280 / 2.0;

                    double error = (tagCenterX - middleScreen) * IN_PER_PIXEL;
                    // this assumes that x axis approaches ∞+ from the left, or left is negative lol

                    switch (place)
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
                    Imgproc.circle(input, new Point(error + middleScreen, tagCenterY - 100), 4, new Scalar(0, 0, 255));

                    // 0.01 ticks = 0.2805 in  trust me lol

                    error = error / 0.2805 * 0.01;

                    target = error + RAIL_CENTER_POS;




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
