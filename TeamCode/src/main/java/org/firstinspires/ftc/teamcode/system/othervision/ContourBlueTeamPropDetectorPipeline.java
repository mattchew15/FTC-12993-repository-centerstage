/*
package org.firstinspires.ftc.teamcode.system.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ContourBlueTeamPropDetectorPipeline extends OpenCvPipeline {
    private Telemetry telemetry;

    public enum TeamPropPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private final Mat
            greyImage = new Mat(),
            filteredImage = new Mat(),
            finalImage = new Mat();

    // Define lower and upper bounds for color filtering
    private final Scalar
            lowerBlue = new Scalar(100, 100, 100), // Adjust as needed
            upperBlue = new Scalar(140, 255, 255),
            green = new Scalar(0, 255, 0);

    // Y threshold and line pos
    private final int
            yThreshold = 350, // Adjust as needed
            lineLength = 480,
            line1 = 210,
            line2 = 320;

    // Point for vertical lines
    private final Point
            line1A = new Point(line1, 0),
            line1B = new Point(line1, lineLength),
            line2A = new Point(line2, 0),
            line2B = new Point (line2, lineLength);

    private final List<MatOfPoint> contours = new ArrayList<>();

    private double cx, cy;

    // Default position is left.
    private volatile TeamPropPosition position = TeamPropPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to grey color space
        Imgproc.cvtColor(input, greyImage, Imgproc.COLOR_RGB2GRAY);

        // Apply a color filter to isolate blue cylinders
        Core.inRange(greyImage, lowerBlue, upperBlue, filteredImage);

        // Iterate through image pixels and set pixels below the Y-coordinate threshold to black
        for (int y = 0; y < greyImage.rows(); y++) {
            for (int x = 0; x < greyImage.cols(); x++) {
                if (y < yThreshold) {
                    greyImage.put(y, x, 0); // Set pixel to black
                }
            }
        }

        // Combine the grayscale and color-filtered images
        Core.bitwise_or(greyImage, filteredImage, finalImage);

        // Draw vertical lines
        Imgproc.line(input, line1A, line1B, green);
        Imgproc.line(input, line2A, line2B, green);

        // Find contours in the filtered image
        Imgproc.findContours(filteredImage, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            // Modify the condition based on the shape of the cylinder
            if (approxCurve.total() >= 5) { // Assume cylinders have at least 5 sides
                // Draw contours for visualization
                Imgproc.drawContours(input, Collections.singletonList(contour), -1, green, 2);
                telemetry.addData("Team Prop", "Found");
                telemetry.update();

                // Calculate moments of the contour
                Moments moments = Imgproc.moments(contour);

                // Calculate the centroid coordinates
                cx = moments.get_m10() / moments.get_m00();
                cy = moments.get_m01() / moments.get_m00();
                Point point = new Point(cx, cy);

                // Determine position
                if (cy > yThreshold) {
                    Imgproc.drawMarker(input, point, green);
                    telemetry.addData("x coordinate", cx);
                    telemetry.addData("y coordinate", cy);
                    telemetry.update();

                    Imgproc.drawMarker(input, new Point(650, 350), green);

                    if (cx < line1) {
                        position = TeamPropPosition.LEFT;
                    } else if ( cx < line2) {
                        position = TeamPropPosition.CENTER;
                    } else {
                        position = TeamPropPosition.RIGHT;
                    }
                }
            }
        }
        telemetry.addData("Position", position);
        telemetry.update();
        return input;
    }

    public TeamPropPosition getPosition()
    {
        return position;
    }
    public double getCx() { return cx;}
    public double getCy() { return cy;}
}

 */