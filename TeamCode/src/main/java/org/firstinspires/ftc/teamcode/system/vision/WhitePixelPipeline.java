package org.firstinspires.ftc.teamcode.system.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class WhitePixelPipeline extends OpenCvPipeline {

    private Mat grayImage;
    private Mat maskedImage;

    private Scalar lowerBound = new Scalar(135); // Adjust as needed
    private Scalar upperBound = new Scalar(255); // Adjust as needed
    private Rect boundingRect;
    private Moments moments;
    private int pos;
    private Mat finalImage;
    private Telemetry telemetry;

    public WhitePixelPipeline() {
        grayImage = new Mat();
        maskedImage = new Mat();
        finalImage = new Mat();
    }

    public WhitePixelPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
        grayImage = new Mat();
        maskedImage = new Mat();
        finalImage = new Mat();
    }


    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to Gray color space
        Imgproc.cvtColor(input, grayImage, Imgproc.COLOR_RGB2GRAY);

        // Apply a color filter to isolate white-like colors
        Core.inRange(grayImage, lowerBound, upperBound, maskedImage);

        // Define the Y-coordinate threshold
        int yThreshold = 350; // Adjust the Y-coordinate threshold as needed

        // Iterate through image pixels and set pixels below the Y-coordinate threshold to black
        for (int y = 0; y < grayImage.rows(); y++) {
            for (int x = 0; x < grayImage.cols(); x++) {
                if (y < yThreshold) {
                    grayImage.put(y, x, 0); // Set pixel to black
                }
            }
        }

        // Combine the color mask and modified grayscale image

        Core.bitwise_and(maskedImage, grayImage, finalImage);

        // Find contours in the filtered image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(maskedImage, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            if (approxCurve.total() == 6 ) { //|| approxCurve.total() <=7
                // Detected hexagon
                // Perform actions or calculations based on hexagon detection
                // For example, you can calculate its position, angle, or control robot movement
                // Draw the detected hexagon on the input frame for visualization
                Imgproc.drawContours(input, Collections.singletonList(contour), -1, new Scalar(0, 255, 0), 2);
                telemetry.addData("[Pixel]", "True");
                telemetry.update();

                // Get the bounding box coordinates
                boundingRect = Imgproc.boundingRect(contour);

                // Extract coordinates
                int x = boundingRect.x;
                int y = boundingRect.y;
                int width = boundingRect.width;
                int height = boundingRect.height;
                telemetry.addData("[x]", x);
                telemetry.addData("[y]", y);


                // Calculate moments of the contour
                moments = Imgproc.moments(contour);

                // Calculate the centroid coordinates
                double cx = moments.get_m10() / moments.get_m00();
                double cy = moments.get_m01() / moments.get_m00();

                if (cy > 350)
                {
                    Imgproc.drawMarker(input, new Point(cx, cy), new Scalar(0, 255, 0));
                    telemetry.addData("[Centroid x]", cx);
                    telemetry.addData("[Centroid y]", cy);

                    Imgproc.drawMarker(input, new Point(650, 350), new Scalar(0, 255, 0));

                    if (cx < 426) {
                        telemetry.addData("[Pixel pos]", "LEFT");
                        pos = 0;
                        telemetry.update();
                    }
                    if (cx >= 426 && cx <= 852) {
                        telemetry.addData("[Pixel pos]", "MIDDLE");
                        pos = 1;
                        telemetry.update();
                    }
                    if (cx > 852) {
                        telemetry.addData("[Pixel pos]", "RIGHT");
                        pos = 2;
                        telemetry.update();
                    }
                    else
                    {
                        pos = 4;
                    }
                }
            }
            telemetry.update();
        }
        return input;


    }
    public int getPos()
    {
        return pos;
    }
}
