package org.firstinspires.ftc.teamcode.system.vision;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.teamPropLocation;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.visiontest.AprilTagPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class PreloadDetection extends AprilTagPipeline
{
    double[] tagList;
    private Point
            regionLeftA,
            regionLeftB,
            regionRightA,
            regionRightB;

    private Mat regionLeftLight, regionRightLight;

    protected enum Place
    {
        LEFT,
        RIGHT,
        MIDDLE,
        NONE
    }
    Place place;

    public PreloadDetection(Telemetry telemetry)
    {
        super(telemetry);

    }

    public PreloadDetection(int purple)
    {
        super();
    }

    @Override
    public void init(Mat frame)
    {
        super.init(frame);
        if (BLUE_AUTO)
        {
            tagList = new double[]{1, 2, 3};
        } else
        {
            tagList = new double[]{4, 5, 6};
        }
        tagList = new double[]{tagList[teamPropLocation]};

        regionLeftA = new Point();
        regionLeftB = new Point();

        regionRightA = new Point();
        regionRightB = new Point();

        regionLeftLight = new Mat();
        regionRightLight = new Mat();

        //Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
        //drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);


    }

    @Override
    public void finalize()
    {
        super.finalize();
    }

    @Override
    public Mat processFrame(Mat input)
    {
        try
        {
            super.processFrame(input);
            ArrayList<AprilTagDetection> detections = getDetectionsUpdate();
            if (detections != null && detections.size() > 0)
            {
                for (AprilTagDetection detection : detections)
                {
                    // tag list should just have one element inside
                    if (detection.id == tagList[0])
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
                        int tagHeight = topY - bottomY;

                        int inclusionZoneWidth = (int) (tagWidth * 1.5);
                        int inclusionZoneHeight = (int) (tagHeight * 1.5);

                        //int exclusionZoneWidth = (int) (tagWidth * 0.28);
                        //int exclusionZoneHeight = (int) (tagHeight * 0.28);

                        Rect leftInclusionZone = new Rect(
                                tagCenterX - inclusionZoneWidth,
                                tagCenterY - 120, inclusionZoneWidth, inclusionZoneHeight
                        );
                        Rect rightInclusionZone = new Rect(
                                tagCenterX + 0,
                                tagCenterY - 120, inclusionZoneWidth, inclusionZoneHeight
                        );

                        //Rect leftExclusionZone = new Rect(tagCenterX - (int) (inclusionZoneWidth * 0.64), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);
                        //Rect rightExclusionZone = new Rect(tagCenterX + (int) (inclusionZoneWidth * 0.28), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);


                        //regionLeftA = new Point(middlePoint.x, middlePoint.y - 200);
                        //regionLeftB = new Point(middlePoint.x - 100, middlePoint.y - 100);

                        //regionRightA = new Point(middlePoint.x, middlePoint.y - 200);
                        //regionRightB = new Point(middlePoint.x + 100, middlePoint.y - 100);

                        // Create area
                        regionLeftLight = input.submat(leftInclusionZone);
                        regionRightLight = input.submat(rightInclusionZone);


                        double leftAvg = Core.mean(regionLeftLight).val[0];
                        double rightAvg = Core.mean(regionRightLight).val[0];

                        if (Math.abs(leftAvg - rightAvg) < 20 && leftAvg < 70)
                        {
                            place = Place.NONE;
                        } else if (Math.abs(leftAvg - rightAvg) < 35 && leftAvg > 100 || Math.abs(leftAvg - rightAvg) < 35 && leftAvg < 100)
                        {
                            place = Place.MIDDLE;
                        } else
                        {
                            place = leftAvg > rightAvg ? Place.LEFT : Place.RIGHT;
                        }

                        Imgproc.rectangle(input, leftInclusionZone, new Scalar(0, 0, 255), 2);
                        Imgproc.rectangle(input, rightInclusionZone, new Scalar(0, 0, 255), 2);

                        switch (place)
                        {
                            case RIGHT:
                                Imgproc.rectangle(input, regionRightA, regionRightB, new Scalar(255, 0, 0), -2);
                                break;
                            case LEFT:
                                Imgproc.rectangle(input, regionLeftA, regionLeftB, new Scalar(255, 0, 0), -2);
                                break;
                            case MIDDLE:
                                Imgproc.rectangle(input, regionRightA, regionRightB, new Scalar(255, 0, 0), -2);
                                Imgproc.rectangle(input, regionLeftA, regionLeftB, new Scalar(255, 0, 0), -2);
                                break;
                            case NONE:
                                break;

                        }
                        if (telemetry != null)
                        {
                            telemetry.addData("LeftAVG", leftAvg);
                            telemetry.addData("RightAVG", rightAvg);
                            telemetry.addData("Difference", Math.abs(leftAvg - rightAvg));

                            telemetry.addData("Place", place);
                        }
                        break;
                        // no need to iterate if you found it
                    }

                }

            }
        } catch (Exception e)
        {

        }
        return input;
    }

    public Place getYellowPixelState()
    {
        return place;
    }
}

