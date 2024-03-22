package org.firstinspires.ftc.teamcode.system.vision;





import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.system.visiontest.AprilTagPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class RelocalizationAprilTagPipeline extends AprilTagPipeline
{
    private final Object object = new Object();
    ArrayList<Integer> tagList = new ArrayList<>();

    AprilTagLibrary library = getCenterStageTagLibrary(); // apparently the official one is wrong

    //Pose2d pos = new Pose2d(0, 0, 0);
    double[] pos;
    // = new double[]{ROBOT_POSE.getX(),ROBOT_POSE.getY(),ROBOT_POSE.getHeading()};

    public static double CAMERA_OFFSET = 4;

    public RelocalizationAprilTagPipeline(Telemetry telemetry)
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
        synchronized (object)
        {
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
    public Pose2d getPos(double angle)
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
        synchronized (object)
        {
            angle = Math.toRadians(angle);
            double X_OFFSET = Math.cos(angle) * CAMERA_OFFSET;
            double Y_OFFSET = Math.sin(angle) * CAMERA_OFFSET;

            return new Pose2d(pos[0] + X_OFFSET, pos[1] + Y_OFFSET, angle);
        }
        //return new double[]{pos[0] + realX, pos[1] + realY};
    }
    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
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

