
package org.firstinspires.ftc.teamcode.system.vision.visionProcessorAPI;


import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.place;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.teamPropLocation;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class PreloadDetectionPipeline implements VisionProcessor
{

//    private AprilTagProcessor aprilTag;
//
//    public PreloadDetectionPipeline(AprilTagProcessor aprilTag) {
//        this.aprilTag = aprilTag;
//    }

    private AprilTagProcessor aprilTagProcessor;
    private Telemetry telemetry;
    private int yOffSet = 90;
    private double i = 2.5;

    public PreloadDetectionPipeline(AprilTagProcessor aprilTag, Telemetry telemetry)
    {
        this.aprilTagProcessor = aprilTag;
        this.telemetry = telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        try
        {
            if (detections != null)
            {
                for (AprilTagDetection detection : detections)
                {
                    if (detection.metadata != null)
                    {
                        if (BLUE_AUTO ? detection.id == 4 - teamPropLocation : detection.id == teamPropLocation + 3)
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

                            // this step is like necessary because int as arguments ig
                            int tagCenterX = (int) detection.center.x;
                            int tagCenterY = (int) detection.center.y;

                            int tagWidth = rightX - leftX;
                            int tagHeight = topY - bottomY;

                            int inclusionZoneWidth = (int) (tagWidth * 1.5);
                            int inclusionZoneHeight = (int) (tagHeight * 1.5);


                            if (tagCenterY - tagHeight * i - inclusionZoneHeight < 0)
                            {
                                i = 2;
                            }
                            yOffSet = (int) (tagHeight * i);


                            //yOffSet = (int) Math.round(1.034 * detection.ftcPose.y + 44.48);
                            Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - yOffSet, inclusionZoneWidth, inclusionZoneHeight);
                            Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - yOffSet, inclusionZoneWidth, inclusionZoneHeight);


                            Imgproc.rectangle(frame, leftInclusionZone, new Scalar(0, 255, 0), 7);
                            Imgproc.rectangle(frame, rightInclusionZone, new Scalar(0, 255, 0), 7);

                            double leftZoneAverage = Core.mean(frame.submat(leftInclusionZone)).val[0];
                            double rightZoneAverage = Core.mean(frame.submat(rightInclusionZone)).val[0];

                            //telemetry.addData("Left zone", leftZoneAverage);
                            //telemetry.addData("Right zone", rightZoneAverage);
                            //telemetry.addData("Diff", Math.abs(leftZoneAverage - rightZoneAverage));
                            place = leftZoneAverage < rightZoneAverage ? Globals.Place.LEFT : Globals.Place.RIGHT; // this should be correct now

                            //Imgproc.rectangle(input, leftInclusionZone, new Scalar(0, 0, 255), 2);
                            //Imgproc.rectangle(input, rightInclusionZone, new Scalar(0, 0, 255), 2);


//                        System.out.println("LEFTAVG " + leftZoneAverage);
//                        System.out.println("RIGHTAVG " + rightZoneAverage);

                        }
                    }
                }
            }
        }
        catch (Exception e)
        {

        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Globals.Place getPreloadYellow()
    {
        return place;
    }

    // i guess we can iterate mannualy through so we can exclude the backdrop areas
    // but i am not sure if that is needed, we can probably assume that the lighting will be equal
    // and both areas will have the same standard value, idk


}
