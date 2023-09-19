package org.firstinspires.ftc.teamcode.system.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedTeamPropDetectorPipeline extends OpenCvPipeline {
    private enum teamPropPosition {
        left,
        center,
        right
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
    private final Point
            region1A = new Point(150, 200),
            region1B = new Point(region1A.x + regionWidth, region1A.y + regionHeight),
            region2A = new Point(300, 100),
            region2B = new Point(region2A.x + regionWidth, region2A.y + regionHeight),
            region3A = new Point(450, 200),
            region3B = new Point(region3A.x + regionWidth, region3A.y + regionHeight);

    private Mat region1Cr, region2Cr, region3Cr;

    private final Mat
            YCrCb = new Mat(),
            Cr = new Mat();

    private int avg1, avg2, avg3;

    private volatile teamPropPosition position = teamPropPosition.left;

    //Take the RGB frame and convert to YCrCb, then extract the Cr channel.
    void inputToCr(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        /*
        To make sure the 'Cr' object is initialized, so the submats will still be linked
        if the object were to only be initialized in processFrame, then the submats would delink
        because the back buffer would be re-allocated the first time a real frame was crunched.
         */

        inputToCr(firstFrame);

        region1Cr = Cr.submat(new Rect(region1A, region1B));
        region2Cr = Cr.submat(new Rect(region2A, region2B));
        region3Cr = Cr.submat(new Rect(region3A, region3B));
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
        After converted to YCrCb, just the Cr channel is extracted because team prop is red.
        Then take the average pixel value of the 3 different regions on that Cr channel.
        We assume the brightest is the team prop.
        Also draw blue rectangles on the screen showing where the regions are, as well as green
        rectangle over the regions believed to be the team prop.
         */

        inputToCr(input);

        //Average pixel value of each Cr channel.
        avg1 = (int) Core.mean(region1Cr).val[0];
        avg2 = (int) Core.mean(region2Cr).val[0];
        avg3 = (int) Core.mean(region3Cr).val[0];

        //Draw rectangles showing regions. Simply visual aid.
        Imgproc.rectangle(input, region1A, region1B, blue, 2);
        Imgproc.rectangle(input, region2A, region2B, blue, 2);
        Imgproc.rectangle(input, region3A, region3B, blue, 2);

        //Find max average, this will be where the team prop is.
        int max = Math.max(avg1, Math.max(avg2, avg3));

        if(max == avg1) {
            position = teamPropPosition.left;
            Imgproc.rectangle(input, region1A, region1B, green, -1);
        } else if(max == avg2) {
            position = teamPropPosition.center;
            Imgproc.rectangle(input, region2A, region2B, green, -1);
        } else {
            position = teamPropPosition.right;
            Imgproc.rectangle(input, region3A, region3B, green, -1);
        }

        return input;
    }

    public teamPropPosition getPosition() { return position; }

    public int getAvg1() { return avg1; }
    public int getAvg2() { return avg2; }
    public int getAvg3() { return avg3; }
}
