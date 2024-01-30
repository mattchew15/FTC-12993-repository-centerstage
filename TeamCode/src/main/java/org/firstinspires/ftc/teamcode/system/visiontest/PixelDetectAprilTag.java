package org.firstinspires.ftc.teamcode.system.visiontest;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class PixelDetectAprilTag extends AprilTagPipeline
{
    public static short pixelAlignment;
    private static final boolean BLUE_SIDE = true;
    int position = 0;
    Tag lookTag;
    double[] tagList;

    public PixelDetectAprilTag(Telemetry telemetry)
        {
            super(telemetry);

        }

        @Override
        public void init(Mat frame)
        {
            super.init(frame);
            if (BLUE_SIDE)
            {
                tagList = new double[]{1,2,3};
            }
            else
            {
                tagList = new double[]{4,5,6};
            }
            tagList = new double[]{tagList[position]};

            //Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
            //drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);


        }

        @Override
        public void finalize()
        {
            super.finalize();
        }

        //@SuppressWarnings("unchecked")
        @Override
        public Mat processFrame(Mat input)
        {
            super.processFrame(input);

            // What the actual fuck i just did?
            ArrayList<ArrayList<?>> tags = processDetections();
            ArrayList<Tag> tags1 = (ArrayList<Tag>) tags.get(0);
            ArrayList<AprilTagDetection> detections1 = (ArrayList<AprilTagDetection>) tags.get(1);

            int i = 0;
            for(Tag dect: tags1)
            {
                if (dect.id == tagList[0])
                {
                    lookTag = dect;
                    break;
                }
                else
                {
                    i++;
                }
            }
            AprilTagDetection myDetection; // wrong scope, fine by now
            if (detections1.size() > 0)
            {
                myDetection = detections1.get(0);
                Pose pose = poseFromTrapezoid(myDetection.corners, cameraMatrix, tagsizeX, tagsizeY);
                Point point = getMiddlePoint(tagsizeY / 2.0, pose.rvec,pose.tvec,cameraMatrix);
                telemetry.addData("Ponto", point);
            }


            telemetry.update();
            return input;
        }
}
