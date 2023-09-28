package org.firstinspires.ftc.teamcode.system.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PixelDetectorPipeline extends OpenCvPipeline {
    private Mat maskedImage;
    private int[][] gameBoard = new int[9][7];  // Matrix to represent the game board
    private int currentColor = 1;  // Initialize with color 1

    @Override
    public Mat processFrame(Mat input) {
        Mat grayImage = new Mat();
        Mat finalImage = new Mat();
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        // Convert input frame to grayscale
        Imgproc.cvtColor(input, grayImage, Imgproc.COLOR_RGB2GRAY);

        // Define lower and upper bounds for color filtering for each color
        Scalar lowerWhite = new Scalar(200, 200, 200);
        Scalar upperWhite = new Scalar(255, 255, 255);
        Scalar lowerGreen = new Scalar(0, 100, 0);
        Scalar upperGreen = new Scalar(100, 255, 100);
        Scalar lowerOrange = new Scalar(150, 100, 0);
        Scalar upperOrange = new Scalar(255, 200, 100);
        Scalar lowerPurple = new Scalar(100, 0, 100);
        Scalar upperPurple = new Scalar(200, 100, 200);

        // Apply color filtering for each color
        Mat whiteMask = new Mat();
        Mat greenMask = new Mat();
        Mat orangeMask = new Mat();
        Mat purpleMask = new Mat();
        Core.inRange(input, lowerWhite, upperWhite, whiteMask);
        Core.inRange(input, lowerGreen, upperGreen, greenMask);
        Core.inRange(input, lowerOrange, upperOrange, orangeMask);
        Core.inRange(input, lowerPurple, upperPurple, purpleMask);

        // Combine the grayscale and color-filtered images
        Core.bitwise_or(grayImage, whiteMask, finalImage);
        Core.bitwise_or(finalImage, greenMask, finalImage);
        Core.bitwise_or(finalImage, orangeMask, finalImage);
        Core.bitwise_or(finalImage, purpleMask, finalImage);

        // Find contours in the filtered image
        Imgproc.findContours(finalImage, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            if (approxCurve.total() == 6) {
                // Draw contours for visualization
                Imgproc.drawContours(input, Collections.singletonList(contour), -1, new Scalar(0, 255, 0), 2);

                // Determine color, row, and column
                int color = determineColor(input, contour);
                int row = determineRow(contour);
                int col = determineColumn(contour);

                if (gameBoard[row][col] == 0) {
                    gameBoard[row][col] = color;
                }
            }
        }

        return input;
    }

    private int determineColor(Mat input, MatOfPoint contour) {
        // Get the bounding box of the contour
        Rect boundingRect = Imgproc.boundingRect(contour);

        // Extract a region of interest (ROI) based on the bounding box
        Mat roi = new Mat(input, boundingRect);

        // Calculate the mean color of the ROI
        Scalar meanColor = Core.mean(roi);

        // Define color ranges for each color
        Scalar whiteLower = new Scalar(200, 200, 200);
        Scalar whiteUpper = new Scalar(255, 255, 255);

        Scalar greenLower = new Scalar(0, 100, 0);
        Scalar greenUpper = new Scalar(100, 255, 100);

        Scalar orangeLower = new Scalar(150, 100, 0);
        Scalar orangeUpper = new Scalar(255, 200, 100);

        Scalar purpleLower = new Scalar(100, 0, 100);
        Scalar purpleUpper = new Scalar(200, 100, 200);

        // Compare the meanColor to the predefined color ranges and return the corresponding color code.

        // Determine white color
        if (isColorInRange(meanColor, whiteLower, whiteUpper)) {
            return 1; // White
        }

        // Determine green color
        if (isColorInRange(meanColor, greenLower, greenUpper)) {
            return 2; // Green
        }

        // Determine orange color
        if (isColorInRange(meanColor, orangeLower, orangeUpper)) {
            return 3; // Orange
        }

        // Determine purple color
        if (isColorInRange(meanColor, purpleLower, purpleUpper)) {
            return 4; // Purple
        }

        return 0; // Return 0 if no color is detected
    }

    private boolean isColorInRange(Scalar color, Scalar lower, Scalar upper) {
        // Check if the given color is within the specified range
        return (color.val[0] >= lower.val[0] && color.val[1] >= lower.val[1] && color.val[2] >= lower.val[2] &&
                color.val[0] <= upper.val[0] && color.val[1] <= upper.val[1] && color.val[2] <= upper.val[2]);
    }

    private int determineRow(MatOfPoint contour) {
        // Calculate the Y-coordinate of the centroid of the contour
        Moments moments = Imgproc.moments(contour);
        double cy = moments.get_m01() / moments.get_m00();

        // You can define thresholds to determine the row based on the Y-coordinate of the centroid.
        // For example, you can divide the Y-coordinate range into zones corresponding to rows on the game board.

        // Example: Determine the row based on the Y-coordinate
        if (cy < 100) {
            return 0; // Top row
        } else if (cy < 200) {
            return 1; // Second row
        }

        // Add similar logic for other rows as needed.

        return 0; // Return 0 if the row cannot be determined
    }

    private int determineColumn(MatOfPoint contour) {
        // Calculate the X-coordinate of the centroid of the contour
        Moments moments = Imgproc.moments(contour);
        double cx = moments.get_m10() / moments.get_m00();

        // You can define thresholds to determine the column based on the X-coordinate of the centroid.
        // For example, you can divide the X-coordinate range into zones corresponding to columns on the game board.

        // Example: Determine the column based on the X-coordinate
        if (cx < 100) {
            return 0; // Leftmost column
        } else if (cx < 200) {
            return 1; // Second column
        }

        // Add similar logic for other columns as needed.

        return 0; // Return 0 if the column cannot be determined
    }

    public int[][] getGameBoard() {
        return gameBoard;
    }
}
