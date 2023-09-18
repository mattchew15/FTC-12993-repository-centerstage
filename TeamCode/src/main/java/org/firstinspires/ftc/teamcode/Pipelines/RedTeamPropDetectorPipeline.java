package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedTeamPropDetectorPipeline extends OpenCvPipeline {
    public enum teamPropPosition {
        left,
        center,
        right
    }

    //Colors for rectangles drawn
    final Scalar blue = new Scalar(0, 0, 255);
    final Scalar green = new Scalar(0, 255, 0);

    //Values for location and size of rectangles.
    final int regionWidth = 40;
    final int regionHeight = 40;
    //Points A and B for 3 regions. Counting from left.
    Point region1A = new Point(150, 200);
    Point region1B = new Point(region1A.x + regionWidth, region1A.y + regionHeight);
    Point region2A = new Point(300, 100);
    Point region2B = new Point(region2A.x + regionWidth, region2A.y + regionHeight);
    Point region3A = new Point(450, 200);
    Point region3B = new Point(region3A.x + regionWidth, region3A.y + regionHeight);

    Mat region1Cr, region2Cr, region3Cr;
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    int avg1, avg2, avg3;

    teamPropPosition position = teamPropPosition.left;

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

        /*
        Submats are a persistent reference to a region of the parent buffer. Any changes to the
        child affect the parent, and vise versa.
         */
        region1Cr = Cr.submat(new Rect(region1A, region1B));
        region2Cr = Cr.submat(new Rect(region2A, region2B));
        region3Cr = Cr.submat(new Rect(region3A, region3B));
    }

    @Override
    public Mat processFrame(Mat input) {
        /*
        First convert to YCrCb color space, from RGB color space. In RGB, the chroma and luma are
        intertwined. In YCrCb, they are separated. YCrCb's 3 channels are
        Y, the luma channel (B&W),
        Cr channel (records the difference from red)
        Cb channel (records the difference from blue)
        Because chroma and luma are not related in YCrCb, vision code written to look for certain
        values in the Cr/Cb channels won't be affected by differing light intensity, since that's
        most in the Y channel.
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

    public void printPosition(Telemetry telemetry) {
        telemetry.addData("Team prop", position);

        telemetry.addData("region1 Cr value", avg1);
        telemetry.addData("region2 Cr value", avg2);
        telemetry.addData("region3 Cr value", avg3);
    }
}
