package org.firstinspires.ftc.teamcode.system.visiontest;





import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;

import java.util.ArrayList;
import java.util.Arrays;

public class RelocalizationAprilTag extends AprilTagPipeline
{

    private final Object object = new Object();
    ArrayList<Integer> tagList = new ArrayList<>();

    AprilTagLibrary library = getCenterStageTagLibrary(); // apparently the official one is wrong

    //Pose2d pos = new Pose2d(0, 0, 0);
    double[] pos = new double[]{0,0,0};

    public static double CAMERA_OFFSET = 4;

    public RelocalizationAprilTag(Telemetry telemetry)
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
                    AprilTagMetadata metadata = library.lookupTag(detection.id);

                    // TODO: after test substitute for real Pose2d
                    double x = metadata.fieldPosition.get(0), y = metadata.fieldPosition.get(1); // get the x, y of the tag in the field
                    pos = new double[]{x + detection.pose.y, y + detection.pose.x, pos[0]}; // heading will always be the imu one
                    break;
                }
            }
        }
        telemetry.addData("New position x", pos[0]);
        telemetry.addData("New position y", pos[1]);
        telemetry.update();
        return input;
    }

    /**
     *
     * @param angle DEGREES IG THEN
     * @return
     */
    public double[] getPos(double angle)
    {
        /*
        -----_-----

               Y offset, sin robot angle
                        ^
                        |
                    _____________
                    |  ___      |
                    |     |     |  -> X offset, cos robot angle
                   ||     *     |
                    |           |
                    |           |
                    -------------

         */
        angle = Math.toRadians(angle);
        double realX = Math.cos(angle) * CAMERA_OFFSET;
        double realY = Math.sin(angle) * CAMERA_OFFSET;
        return new double[]{pos[0] + realX, pos[1] + realY};
    }
}

/*
     public VectorF(float x, float y, float z, float w
        {
        this.data = new float[4];
        this.data[0] = x;
        this.data[1] = y;
        this.data[2] = z;
        this.data[3] = w;
        }
 */

