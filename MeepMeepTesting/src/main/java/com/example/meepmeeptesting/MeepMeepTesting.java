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

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -59, Math.toRadians(180)))
                                //.splineToSplineHeading(new Pose2d(38.5,-25, Math.toRadians(90)),Math.toRadians(90))
                                //.splineToLinearHeading(new Pose2d(38.5, -25, Math.toRadians(0)), Math.toRadians(90))
                                //.lineTo(new Vector2d(38.5, -40))
                                //.splineToConstantHeading(new Vector2d(38,-25) ,Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(35, -32, Math.toRadians(175)))
                               // .splineToSplineHeading(new Pose2d(26, -36, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(26, -36), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(20, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-37, -36, Math.toRadians(180))) // slower portion of spline
                                //
                                .lineToLinearHeading(new Pose2d(27, -36, Math.toRadians(180)))
                                .lineTo(new Vector2d(32, -36))
                                .lineTo(new Vector2d(32, -35)) // seperates trajectories
                                //
                                .splineToConstantHeading(new Vector2d(26, -36 + 15), Math.toRadians(175))
                                .lineToSplineHeading(new Pose2d(20, -36 + 15, Math.toRadians(175)))
                                .lineToSplineHeading(new Pose2d(-37, -36 + 15, Math.toRadians(180))) // slower portion of spline



                                //.splineToConstantHeading(new Vector2d(25,-18.7) ,Math.toRadians(180))
                                //.splineToSplineHeading(new Pose2d(17, -18.7, Math.toRadians(3)), Math.toRadians(180))

                                //   .lineTo(new Vector2d(-27, -17))
                                //  .splineToSplineHeading(new Pose2d(-42,-17.7, Math.toRadians(180)),Math.toRadians(180))

                                //.lineTo(new Vector2d(45,-14))
                               // .splineToSplineHeading(new Pose2d(-8,-18, Math.toRadians(177)),Math.toRadians(180))
                               // .splineToSplineHeading(new Pose2d(-37.37,-18.85, Math.toRadians(177)),Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
