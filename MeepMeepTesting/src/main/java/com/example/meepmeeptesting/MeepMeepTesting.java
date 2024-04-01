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
        double frontOffset = 12;
        Pose2d stacks = new Pose2d(-28,-36,Math.toRadians(180));
        Pose2d backdrop = new Pose2d(28,-36,Math.toRadians(180)); // not necessary lol
        Pose2d startPoseFront = new Pose2d(-38, -59, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseFront)

                                .lineToLinearHeading(new Pose2d(-50, -32, Math.toRadians(180)))

                                .splineToConstantHeading(new Vector2d(-40, (-28 - 22)), Math.toRadians(-35))
                                .splineToConstantHeading(new Vector2d(-18, (-32 - 22)), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(10, -32 - 22, Math.toRadians(0)))
                                .splineTo(new Vector2d(32,-28 - 22), Math.toRadians(14))
                                /*
                                .lineToSplineHeading(new Pose2d(24.1, (-32 + 4), Math.toRadians(179)))
                                .splineToConstantHeading(new Vector2d(20, (-32 + 10)), Math.toRadians(110))
                                .splineToConstantHeading(new Vector2d(6, (-32 + 26.2)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(5, (-32 + 26.2), Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-27, (-32 + 26.2), Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-37, (-32 + 26.2), Math.toRadians(180))) // slower portion of spline
                                 */
                                /*
                                .lineToSplineHeading(new Pose2d(20, -32, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-28, -29), Math.toRadians(180)) // end tangent affects path alot\
                                .lineToSplineHeading(new Pose2d(-36.7, -29, Math.toRadians(180))) // seperates trajectories

                                .splineToConstantHeading(new Vector2d(15, -36),Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(10, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(180)))

                                .lineToSplineHeading(new Pose2d(-8, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(15, -36, Math.toRadians(-160)))
                                .splineToConstantHeading(new Vector2d(31, -36 +6),Math.toRadians(20))

                                */
/*
                                .splineToConstantHeading(new Vector2d(15, -36),Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(10, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-15, -36, Math.toRadians(180)))
                                //.lineToSplineHeading(new Pose2d(-18, -36, Math.toRadians(160)))
                                .splineToSplineHeading(new Pose2d(-36,-32, Math.toRadians(160)), Math.toRadians(170))
                                //.splineToConstantHeading(new Vector2d(-36, -32),Math.toRadians(160))

 */
                                // Below is the code for going into the stacks at an angle
/*
                                .splineTo(new Vector2d(15,-36), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-15, -36, Math.toRadians(180)))
                                .splineTo(new Vector2d(-36,-33), Math.toRadians(160))
                                */


                                // Two types of end splines where we turn
                                // The one below slowly turns first, then strafes towards backdrop slightly
                                // Other spline turns slightly and drives

                                // type one
                                /*
                                .splineTo(new Vector2d(-18,-36), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(-16, -36, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(16, (-36), Math.toRadians(5)))
                                .splineToConstantHeading(new Vector2d(32, (-32)),Math.toRadians(15))
                                 */

                                // type 2
                             /*   .splineTo(new Vector2d(-18,-36), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(10, -36, Math.toRadians(0)))
                                .splineTo(new Vector2d(32,-32), Math.toRadians(14))*/




/*
                                .lineToSplineHeading(new Pose2d(-8, (-32), Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(10, (-32), Math.toRadians(175)),Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(15, (-32), Math.toRadians(175)))
                                .splineToConstantHeading(new Vector2d(32, (-32 -5)),Math.toRadians(-20))
 */


                                .build()
                      
/*
                                .splineToConstantHeading(new Vector2d(15, -36),Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(10, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(180)))

                                .lineToSplineHeading(new Pose2d(-8, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(15, -36, Math.toRadians(-160)))
                                .splineToConstantHeading(new Vector2d(31, -36 +6),Math.toRadians(20))

                                .splineToConstantHeading(new Vector2d(15, -36),Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(10, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-15, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-18, -36, Math.toRadians(160)))
                                .splineToConstantHeading(new Vector2d(-36, -32),Math.toRadians(160))

                                .splineToConstantHeading(new Vector2d(-18, -36),Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(0, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(20, -36, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(24, -36, Math.toRadians(-160)))
                                .splineToConstantHeading(new Vector2d(36, -32),Math.toRadians(20))
 */

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
