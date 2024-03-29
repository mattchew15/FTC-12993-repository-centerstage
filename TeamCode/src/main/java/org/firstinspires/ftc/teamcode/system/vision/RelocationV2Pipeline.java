package org.firstinspires.ftc.teamcode.system.vision;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.getCenterStageTagLibrary;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.AprilTagLocalisation;
import org.firstinspires.ftc.teamcode.system.visiontest.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.system.visiontest.AprilTagPipelineOld;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class RelocationV2Pipeline extends AprilTagPipeline
{

    Telemetry telemetry;
    private final Object object = new Object();
    ArrayList<Integer> tagList = new ArrayList<>();
    private double poseX, poseY;

    AprilTagLibrary library = getCenterStageTagLibrary(); // apparently the official one is wrong

    //Pose2d pos = new Pose2d(0, 0, 0);
    double[] pos = new double[]{0,0};
    // = new double[]{ROBOT_POSE.getX(),ROBOT_POSE.getY(),ROBOT_POSE.getHeading()};

    public static double CAMERA_OFFSET = 6.15;
    private AprilTagLocalisation aprilTagLocalisation = new AprilTagLocalisation();

    public RelocationV2Pipeline(Telemetry telemetry)
    {
        super(telemetry);

    }
    public RelocationV2Pipeline()
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
                            telemetry.addLine("we're here");
                            AprilTagMetadata metadata = library.lookupTag(detection.id);

                            // TODO: after test substitute for real Pose2d
                            double x = metadata.fieldPosition.get(0), y = metadata.fieldPosition.get(1); // get the x, y of the tag in the field
                            pos = new double[]{x, y}; // heading will always be the imu one
                            poseX = detection.pose.x;
                            poseY = detection.pose.y;

                            break;
                        }
                    }
                }


            }

            telemetry.addData("Pose y", poseY);
            telemetry.addData("Pose x", poseX);
            telemetry.addData("AprilTagPos x", pos[0]);
            telemetry.addData("AprilTagPos y", pos[1]);
            telemetry.update();


            telemetry.update();
        }
        catch (Exception e)
        {
            //telemetry.addData("Error", e.getStackTrace());
            telemetry.addData("FUCK", true);
            telemetry.update();
            e.printStackTrace();
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

        try
        {
            // todo remove sync
            synchronized (object)
            {
                // double X_OFFSET = Math.cos(angle) * CAMERA_OFFSET;
                //double Y_OFFSET = Math.sin(angle) * CAMERA_OFFSET;
                Pose2d newPose = aprilTagLocalisation.getCorrectedPose(angle, poseX, poseY);
                telemetry.addData("Lotus X", newPose.getX());
                telemetry.addData("Lotus Y", newPose.getY());
                return new Pose2d(pos[0] + newPose.getX(), pos[1] + newPose.getY(), angle);
            }
        }
        catch (Exception e)
        {

            //telemetry.addData("Error", e.getStackTrace());
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
