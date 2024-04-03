package org.firstinspires.ftc.teamcode.system.vision;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.place;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.teamPropLocation;

import android.graphics.Canvas;

import androidx.core.math.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class RailExtensionPipeline implements VisionProcessor
{


    AprilTagProcessor processor;
    Telemetry telemetry;
    double target = 0.49;
    public RailExtensionPipeline(AprilTagProcessor processor, Telemetry telemetry)
    {
        this.processor = processor;
        this.telemetry = telemetry;
    }
    private final double MAX_RAIL = 11.38; // inches c
    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        List<AprilTagDetection> detections = processor.getDetections();
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    if (BLUE_AUTO ? detection.id == 4 - teamPropLocation : detection.id == teamPropLocation + 3)
                    {
                        double x = detection.ftcPose.x;
                        MathUtils.clamp(x, -MAX_RAIL, MAX_RAIL);
                        telemetry.addData("Error (In)", x);
                        x = errorToTickPosition(x);
                        telemetry.addData("Ticks raw target", x);
                        switch (place)
                        {
                            case NONE:
                                if (teamPropLocation == 1 || teamPropLocation == 4)
                                {
                                    place = Globals.Place.LEFT;
                                } else if (teamPropLocation == 3 || teamPropLocation == 6)
                                {
                                    place = Globals.Place.RIGHT;
                                }
                                // the no break is intentional lol

                            case LEFT:
                                x += 0.05950727972 ; // positive servo ticks
                                break;
                            case MIDDLE:
                                x += 0;
                                break;
                            case RIGHT:
                                x -= 0.05950727972; // negative servo ticks
                                break;
                        }

                        telemetry.addData("Target offseted", x);
                        // 5.91 * Math.PI * 2 / 355
                        target = x;
                    }
                }
            }
        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {

    }

    public double getTarget()
    {
        return target;
    }

    public double errorToTickPosition(double in)
    {
        if (in >= MAX_RAIL)
        {
            return 0;
        } else if (in <= -MAX_RAIL)
        {
            return 1;
        }

        return in < 0 ? 0.49 + Math.abs(InchesToTicksInches(in)) : 0.49 - Math.abs(InchesToTicksInches(in));
    }

    public double InchesToTicksInches(double inches)
    {
        return inches / 25.207; // 14.027; //* 0.01;
    }
}
