package org.firstinspires.ftc.teamcode.system.visiontest;

//import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;
//import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_POS;
//import static org.firstinspires.ftc.teamcode.system.hardware.Globals.place;

//import androidx.core.math.MathUtils;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;



import java.util.ArrayList;

public class RailAdjustAprilTag extends AprilTagPipeline
{
    //AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics().build();

    public enum Place
    {
        LEFT,
        RIGHT,
        MIDDLE,
        NONE
    }
    public static Place depositPlace = Place.LEFT;
    public static double tag = 5;

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
        //TODO: acount for purple idk
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
                if (detection.id == tag)
                {
                    double error = detection.pose.x;
                    switch (depositPlace)
                    {
                        // case off set inches
                        case LEFT:
                            error += 1.5;
                            break;
                        case RIGHT:
                            error -= 1.5;
                            break;
                        case MIDDLE:
                            error += 0;
                            break;
                    }
                    // Imgproc.circle(input, new Point(error + tagCenterX, tagCenterY - 100), 4, new Scalar(0, 0, 255), 4);
                    Imgproc.drawMarker(input, new Point(640, 360), new Scalar(255, 0, 0), 2, 15);
                    // here we have the error in inches
                    telemetry.addData("X inches", error);
                    // 0.01 ticks = 0.2805 in  trust me lol
                    target = error;

                    Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);


                    telemetry.addData("FINAL", getTarget(rot.firstAngle, 2.5));
                    break;
                }
            }
        }
        telemetry.addData("Target", target);
        telemetry.update();
        return input;
    }
    public double InchesToTicksInches(double inches)
    {
       return inches / 14.027 * 0.01;
    }

    public double getTarget(double angle, double liftDis)
    {
        double offset = 2;
        return InchesToTicksInches(target + (liftDis - offset) * Math.sin(angle)) + 0.49;
    }
}




