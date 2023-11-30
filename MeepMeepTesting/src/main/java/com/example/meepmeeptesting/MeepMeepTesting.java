package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -59, Math.toRadians(180)))



                                .lineToSplineHeading(new Pose2d(0, -36, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(5, -36, Math.toRadians(180)), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(27, -36, Math.toRadians(160)))
                                .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(160))) // slower portion of spline

                                .lineToSplineHeading(new Pose2d(-20, -36, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-30, -36), Math.toRadians(180)) // end tangent affects path alot\
                                .lineTo(new Vector2d(-37, -36)) // seperates trajectories

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
