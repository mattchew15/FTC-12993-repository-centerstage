package org.firstinspires.ftc.teamcode.system.vision.visionProcessorAPI;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_POS;
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

import java.util.List;

public class RailExtensionPipelineTemp implements VisionProcessor
{


    AprilTagProcessor processor;
    Telemetry telemetry;
    double target = 0.49;
    double BACKDROP_SHIFT = 0.23;
    double defaultTehee = RAIL_CENTER_POS;
    public RailExtensionPipelineTemp(AprilTagProcessor processor, Telemetry telemetry)
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
        double tagWeSee = 0;
        double targetTag = BLUE_AUTO ? 4 - teamPropLocation : teamPropLocation + 3;
        AprilTagDetection detection = null;
        if (detections != null)
        {
            for (AprilTagDetection dect : detections)
            {
                if (dect.id == targetTag)
                {
                    tagWeSee = targetTag;
                    detection = dect;
                    break;
                }
                else if (dect.id == 2 || dect.id == 5)
                {
                    tagWeSee = dect.id;
                    detection = dect;
                }
                else
                {
                    tagWeSee = dect.id;
                    detection = dect;
                }
            }
            if (detection != null) // this is ugly but like necessary
            {
                if (detection.metadata != null)
                {
                    double x = detection.ftcPose.x;
                    //MathUtils.clamp(x, -MAX_RAIL, MAX_RAIL);
                    telemetry.addData("Error (In)", x);
                    x = errorToTickPosition(x);
                    telemetry.addData("Ticks raw target", x);
                    switch (place)
                    {
                        case MIDDLE:
                            if (targetTag == 1 || targetTag == 4)
                            {
                                place = Globals.Place.LEFT;
                            } else if (targetTag == 3 || targetTag == 6)
                            {
                                place = Globals.Place.RIGHT;
                            }
                            // the no break is intentional lol

                        case LEFT:
                            x += InchesToTicksInches(1.5); // positive servo ticks
                            break;
                        case NONE:
                            x += 0;
                            break;
                        case RIGHT:
                            x -= InchesToTicksInches(1.5); // negative servo ticks
                            break;
                    }
                    //x += InchesToTicksInches(0.2);
                    telemetry.addData("Target Tag", targetTag);
                    telemetry.addData("Seen Tag", tagWeSee);
                    telemetry.addData("Target offseted", x);
                    // 5.91 * Math.PI * 2 / 355
                    double d = 0;
                    if (tagWeSee != 0)
                    {
                         d = (tagWeSee - targetTag) * BACKDROP_SHIFT;
                    }
                    x += d;
                    target = x;
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
    public void setDefault(double defaultTehee)
    {
        this.defaultTehee = defaultTehee;
    }
}
