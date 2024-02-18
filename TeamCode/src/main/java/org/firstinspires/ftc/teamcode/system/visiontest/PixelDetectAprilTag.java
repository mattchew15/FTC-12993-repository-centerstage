package org.firstinspires.ftc.teamcode.system.visiontest;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class PixelDetectAprilTag extends AprilTagPipeline
{
    public static short pixelAlignment;
    private static final boolean BLUE_SIDE = true;
    int position = 0;
    Tag lookTag;
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
        RIGHT
    }


    public PixelDetectAprilTag(Telemetry telemetry)
        {
            super(telemetry);

        }

        @Override
        public void init(Mat frame)
        {
            super.init(frame);
            if (BLUE_SIDE)
            {
                tagList = new double[]{1,2,3};
            }
            else
            {
                tagList = new double[]{4,5,6};
            }
            tagList = new double[]{tagList[position]};

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
            super.processFrame(input);
            ArrayList<AprilTagDetection> detections = getDetectionsUpdate();
            Place place;
            Point middlePoint;
            if (detections != null && detections.size() > 0)
            {
                for (AprilTagDetection detection : detections)
                {
                    if (detection.id == tagList[0])
                    {
                        middlePoint = detection.center;

                        int leftX = Integer.MAX_VALUE;
                        int rightX = Integer.MIN_VALUE;
                        int topY = Integer.MIN_VALUE;
                        int bottomY = Integer.MAX_VALUE;

                        for (Point point : detection.corners) {
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

                        int exclusionZoneWidth = (int) (tagWidth * 0.28);
                        int exclusionZoneHeight = (int) (tagHeight * 0.28);

                        Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);
                        Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);


                        // i didnt find necessary to exclude any zone just parse the whole rect
                        Rect leftExclusionZone = new Rect(tagCenterX - (int) (inclusionZoneWidth * 0.64), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);
                        Rect rightExclusionZone = new Rect(tagCenterX + (int) (inclusionZoneWidth * 0.28), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);


                        //regionLeftA = new Point(middlePoint.x, middlePoint.y - 200);
                        //regionLeftB = new Point(middlePoint.x - 100, middlePoint.y - 100);

                        //regionRightA = new Point(middlePoint.x, middlePoint.y - 200);
                        //regionRightB = new Point(middlePoint.x + 100, middlePoint.y - 100);

                        // Create area
                        regionLeftLight = input.submat(leftInclusionZone);
                        regionRightLight = input.submat(rightInclusionZone);


                        double leftAvg = Core.mean(regionLeftLight).val[0];
                        double rightAvg = Core.mean(regionRightLight).val[0];

                        place = leftAvg > rightAvg ? Place.LEFT : Place.RIGHT;


                        Imgproc.rectangle(input, leftInclusionZone, new Scalar(0, 0, 255), 2);
                        Imgproc.rectangle(input, rightInclusionZone, new Scalar(0, 0, 255), 2);

                        if (place == Place.LEFT)
                        {
                            Imgproc.rectangle(input, regionLeftA, regionLeftB, new Scalar(255, 0, 0), -2);
                        } else
                        {
                            Imgproc.rectangle(input, regionRightA, regionRightB, new Scalar(255, 0, 0), -1);
                        }

                        telemetry.addData("Place", place);
                        break;
                        // no need to iterate if you found it
                    }

                }
            }
            telemetry.update();
            return input;
        }
}
