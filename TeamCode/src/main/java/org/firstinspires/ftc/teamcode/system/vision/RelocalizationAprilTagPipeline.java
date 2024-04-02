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

@Deprecated
public class RelocalizationAprilTagPipeline extends AprilTagPipeline
{
    private final Object object = new Object();
    ArrayList<Integer> tagList = new ArrayList<>();

    AprilTagLibrary library = getCenterStageTagLibrary(); // apparently the official one is wrong

    //Pose2d pos = new Pose2d(0, 0, 0);
    double[] pos = new double[]{0,0,0};
    // = new double[]{ROBOT_POSE.getX(),ROBOT_POSE.getY(),ROBOT_POSE.getHeading()};

    public static double CAMERA_OFFSET = 6.15;

    public RelocalizationAprilTagPipeline(Telemetry telemetry)
    {
        super(telemetry);

    }
    public RelocalizationAprilTagPipeline()
    {
        super();

    }

    @Override
    public void init(Mat frame)
    {
        super.init(frame);
        if (false)
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
        double posey = 0, posex = 0;
        try
        {
            super.processFrame(input);
            ArrayList<AprilTagDetection> detections = getDetectionsUpdate();
            synchronized (object)
            {
                if (detections != null && detections.size() > 0)
                {
                    for (AprilTagDetection detection : detections)
                    {
                        if (detection.id == 5)
                        {
                            AprilTagMetadata metadata = library.lookupTag(detection.id);

                            // TODO: after test substitute for real Pose2d
                            double x = metadata.fieldPosition.get(0), y = metadata.fieldPosition.get(1); // get the x, y of the tag in the field
                            posex = detection.pose.x * FEET_PER_METER * 12;
                            posey = detection.pose.y * FEET_PER_METER * 12;
                            pos = new double[]{x - posey, y + posex, pos[0]}; // heading will always be the imu one
                            break;
                        }
                    }
                }
            }

            telemetry.addData("Pose y", posey);
            telemetry.addData("Pose x", posex);
            telemetry.addData("New position x", pos[0]);
            telemetry.addData("New position y", pos[1]);
            telemetry.update();
        }
        catch (Exception e)
        {
            telemetry.addData("FUCK", true);
            telemetry.update();
        }
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
        try
        {
            // todo remove sync
            synchronized (object)
            {
                angle = Math.toRadians(angle);
               // double X_OFFSET = Math.cos(angle) * CAMERA_OFFSET;
                //double Y_OFFSET = Math.sin(angle) * CAMERA_OFFSET;

                return new Pose2d(pos[0] + 0, pos[1] + 0, angle);
            }
        }
        catch (Exception e)
        {

            telemetry.addData("Error", e.getStackTrace());
            telemetry.addData("FUCK 2", true);
            telemetry.update();

            return new Pose2d(0, 0, 0);
        }
        //return new double[]{pos[0] + realX, pos[1] + realY};
    }
    public void setTelemetry(Telemetry telemetry)
    {
        this.telemetry = telemetry;
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

