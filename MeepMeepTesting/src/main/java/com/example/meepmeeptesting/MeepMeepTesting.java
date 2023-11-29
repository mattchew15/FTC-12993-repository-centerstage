package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        double frontOffset = 23;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -59, Math.toRadians(180)))


                                .lineToLinearHeading(new Pose2d(35, -32, Math.toRadians(175)))
                               //
                               .splineToConstantHeading(new Vector2d(34, -33), Math.toRadians(-110)) // end tangent affects path alot
                               .splineToConstantHeading(new Vector2d(23, -36), Math.toRadians(180))
                               .lineToSplineHeading(new Pose2d(-20, -36, Math.toRadians(180)))
                               .lineToSplineHeading(new Pose2d(-37, -36, Math.toRadians(180))) // slower portion of spline
                               //
                               .lineToLinearHeading(new Pose2d(27, -36, Math.toRadians(180)))
                               .lineTo(new Vector2d(30, -36))
                               .lineTo(new Vector2d(32, -36)) // seperates trajectories
                               //
                               .lineToSplineHeading(new Pose2d(32, -36 + 4, Math.toRadians(175)))
                               .splineToConstantHeading(new Vector2d(32, -36 + 6), Math.toRadians(90))
                               .splineToConstantHeading(new Vector2d(25, -36 + frontOffset), Math.toRadians(180))
                               .lineToSplineHeading(new Pose2d(20, -36 + frontOffset, Math.toRadians(180)))
                               .lineToSplineHeading(new Pose2d(-30, -36 + frontOffset, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-37, -36 + frontOffset, Math.toRadians(180)))
                              //  .splineTo(new Vector2d(-37, -38 + frontOffset), Math.toRadians(-160))
                               //
                               .lineToLinearHeading(new Pose2d(27, -36 + frontOffset, Math.toRadians(180)))
                               .lineTo(new Vector2d(25, -36 + frontOffset))
                               // this one should be reversed
                               //.splineTo(new Vector2d(35, -36 + frontOffset), Math.toRadians(160))





                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
