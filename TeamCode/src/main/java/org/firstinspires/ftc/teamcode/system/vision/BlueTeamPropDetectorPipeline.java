package org.firstinspires.ftc.teamcode.system.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueTeamPropDetectorPipeline extends OpenCvPipeline {
    private enum TeamPropPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Colors for rectangles drawn
    private final Scalar
            blue = new Scalar(0, 0, 255),
            green = new Scalar(0, 255, 0);

    // Values for location and size of rectangles.
    private final int
            regionWidth = 40,
            regionHeight = 40;

    // Points A and B for 3 regions. Counting from left.
    private final Point // position of rectangles
            region1A = new Point(150, 200),
            region1B = new Point(region1A.x + regionWidth, region1A.y + regionHeight),
            region2A = new Point(300, 100),
            region2B = new Point(region2A.x + regionWidth, region2A.y + regionHeight),
            region3A = new Point(450, 200),
            region3B = new Point(region3A.x + regionWidth, region3A.y + regionHeight);

    private Mat region1Cb, region2Cb, region3Cb; // CB values in 3 rectangles

    private final Mat
            YCrCb = new Mat(),
            Cb = new Mat();

    private int avg1, avg2, avg3; // average cb values in each rectangle

    private volatile TeamPropPosition position = TeamPropPosition.LEFT; //default position is left

    // Take the RGB frame and convert to YCrCb, then extract the Cb channel.
    private void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        /*
        To make sure the 'Cb' object is initialized.
        If the object were to only be initialized in processFrame, then the submats would delink
        Because the back buffer would be re-allocated the first time a real frame was crunched.
         */

        inputToCb(firstFrame);

        region1Cb = Cb.submat(new Rect(region1A, region1B));
        region2Cb = Cb.submat(new Rect(region2A, region2B));
        region3Cb = Cb.submat(new Rect(region3A, region3B));
    }

    @Override
    public Mat processFrame(Mat input) {
        /*
        Convert to YCrCb color space from RGB color space, since in RGB the chroma and luma are
        intertwined. In YCrCB, they are separated in 3 channels:
        Y, Luma channel (B&W),
        Cr channel (records the difference from red),
        Cb channel (records the difference from blue).
        Because chroma and luma are not related in YCrCb, values in the Cr/Cb channels won't be
        affected by differing light intensity.
        After converted to YCrCb, just the Cb channel is extracted because team prop is blue.
        Then take the average pixel value of the 3 different regions on that Cb channel.
        We assume the brightest is the team prop.
        Also draw blue rectangles on the screen showing where the regions are, as well as green
        rectangle over the regions believed to be the team prop.
         */

        inputToCb(input);

        //Average pixel value of each Cb channel.
        avg1 = (int) Core.mean(region1Cb).val[0];
        avg2 = (int) Core.mean(region2Cb).val[0];
        avg3 = (int) Core.mean(region3Cb).val[0];

        //Draw rectangles showing regions. Simply visual aid.
        Imgproc.rectangle(input, region1A, region1B, blue, 2);
        Imgproc.rectangle(input, region2A, region2B, blue, 2);
        Imgproc.rectangle(input, region3A, region3B, blue, 2);

        //Find max average, this will be where the team prop is.
        int max = Math.max(avg1, Math.max(avg2, avg3));

        if (max == avg1) {
            position = TeamPropPosition.LEFT;
            Imgproc.rectangle(input, region1A, region1B, green, -1);
        } else if (max == avg2) {
            position = TeamPropPosition.CENTER;
            Imgproc.rectangle(input, region2A, region2B, green, -1);
        } else {
            position = TeamPropPosition.RIGHT;
            Imgproc.rectangle(input, region3A, region3B, green, -1);
        }

        return input;
    }

    public TeamPropPosition getPosition() { return position; }

    public int getAvg1() { return avg1; }
    public int getAvg2() { return avg2; }
    public int getAvg3() { return avg3; }
}
