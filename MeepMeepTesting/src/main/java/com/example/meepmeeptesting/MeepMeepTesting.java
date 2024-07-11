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
        Pose2d stacks = new Pose2d(36,-21,Math.toRadians(180));
        Pose2d backdrop = new Pose2d(28,-36,Math.toRadians(180)); // not necessary lol
        Pose2d startPoseFront = new Pose2d(-38, -59, Math.toRadians(180));
        Pose2d thirdDriveStage = new Pose2d(32, -14.5, Math.toRadians(180));

        // front side truss auto first drive endings:

        Pose2d teamProp2 =new Pose2d(36,-32.8, Math.toRadians(180));
        Pose2d teamProp1 = new Pose2d(36,-32.8 + 3.5, Math.toRadians(180));
        Pose2d teamProp3 =new Pose2d(36,-32.8 - 15, Math.toRadians(180));


        Pose2d backteamProp1 = new Pose2d(36,-29, Math.toRadians(180));
        Pose2d backteamProp2 = new Pose2d(36,-26, Math.toRadians(174));
        Pose2d backteamProp3 = new Pose2d(36,-34, Math.toRadians(180));
        Pose2d afterDrive3 = new Pose2d(-43, -24, Math.toRadians(180));
        Pose2d afterDrive2 = new Pose2d(-49, -17, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(backdrop)
//                                .forward(10)
//                                .turn(Math.toRadians(90))

                                // drive to backdrop after firstdrive3
//                                .lineToSplineHeading(new Pose2d(-41, -15, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(-28, (-9)), Math.toRadians(0))
//                                .lineToSplineHeading(new Pose2d(20, -9, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(36, (-9 - 20)), Math.toRadians(-58))

                                // same for 2 as above

//                                .lineToSplineHeading(new Pose2d(-41, -12, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(-28, (-9)), Math.toRadians(0))
//                                .lineToSplineHeading(new Pose2d(20, -9, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(36, (-9 - 15)), Math.toRadians(-42))


                                //.splineToConstantHeading(new Vector2d(12, 30),Math.toRadians(180))
                                //.splineToConstantHeading(new Vector2d(24, (-30 + 10 + 8)), Math.toRadians(110))
                                //.splineToConstantHeading(new Vector2d(5, (-6)), Math.toRadians(180))
                                //.lineToSplineHeading(new Pose2d(-17, (-6), Math.toRadians(180)))
                                //.lineToSplineHeading(new Pose2d(-27, (-6), Math.toRadians(180)))

                            // drive into stacks truss teamprop3

                                .lineToSplineHeading(new Pose2d(27.5, -47.8 + 5.3, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(10, -52.8), Math.toRadians(180)) // end tangent affects path alot\
                                //.lineToSplineHeading(new Pose2d(-16.5, -53, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(-36,-51, Math.toRadians(160)), Math.toRadians(170))


                                //.splineTo(new Vector2d(-36,-47.8), Math.toRadians(155)) // roughly 160

                                /*.addSpatialMarker(new Vector2d(-24.5, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), () -> {
                                  extendSlidesAroundTruss = true;
                              })*/
                                /* .addSpatialMarker(new Vector2d(-11, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), () -


                                //.lineToSplineHeading(new Pose2d(35, -36, Math.toRadians(180)))
                                //.splineToConstantHeading(new Vector2d(-16.5, -36), Math.toRadians(180)) // end tangent affects path alot\

                                /*.addSpatialMarker(new Vector2d(-24.5, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), () -> {
                                    extendSlidesAroundTruss = true;
                                })*/
                                /* .addSpatialMarker(new Vector2d(-11, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), () -> {
                                     extendSlidesAroundStage = true;
                                 })

                                 */
                                //.splineTo(new Vector2d(-36,-32), Math.toRadians(155)) // roughly 160

//                                .lineToSplineHeading(new Pose2d(28, (-28), Math.toRadians(-5)))
//
//                                .lineToSplineHeading(new Pose2d(32, -28, Math.toRadians(-5)))
/*
                                .lineToSplineHeading(new Pose2d(22, -12, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(22, -12), Math.toRadians(-7)) // end tangent of path*/

                                /*.lineToSplineHeading(new Pose2d(27, -28, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-10, -28), Math.toRadians(180)) // end tangent affects path alot\
                                .lineToSplineHeading(new Pose2d(-26, -28, Math.toRadians(180))) // seperates trajectories*/



//                                .lineToSplineHeading(new Pose2d(20, -9, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(36, (-9 - 7)), Math.toRadians(-38))

                                /*
                                .lineToLinearHeading(new Pose2d(-50, -32, Math.toRadians(180)))
                                // truss first spline for middle yellow

                                .splineToConstantHeading(new Vector2d(-40, (-28 - 22)), Math.toRadians(-35))
                                .splineToConstantHeading(new Vector2d(-18, (-32 - 22)), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(-15, -32 - 22, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(36, (-18 - 22)), Math.toRadians(55))
*/

                                // -29 - back stage 2
                             /*   .lineToSplineHeading(new Pose2d(33, (-32 + 11), Math.toRadians(180)))
                                //.splineToConstantHeading(new Vector2d(33, (-32 + 11)), Math.toRadians(110))
                                .splineToConstantHeading(new Vector2d(5, (-32 + 26.2)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-27, (-32 + 26.2), Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-37, (-32 + 26.2), Math.toRadians(180)))*/

                                // -21
                               /* .splineToConstantHeading(new Vector2d(23, (-32 + 10 + 8)), Math.toRadians(110))
                                .splineToConstantHeading(new Vector2d(5, (-32 + 26.2)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-17, (-32 + 26.2), Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-27, (-32 + 26.2), Math.toRadians(180)))*/

                                //-15
                              /*  .splineToConstantHeading(new Vector2d(24, (-32 + 25)), Math.toRadians(170))
                                .splineToConstantHeading(new Vector2d(10, (-32 + 26.2)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-27, (-32 + 26.2), Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-37, (-32 + 26.2), Math.toRadians(180)))*/
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

//                                .splineTo(new Vector2d(15,-36), Math.toRadians(180))
//                                .lineToSplineHeading(new Pose2d(-15, -36, Math.toRadians(180)))
//                                .splineTo(new Vector2d(-36,-33), Math.toRadians(160))



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
                       /*        .splineTo(new Vector2d(-18,-36), Math.toRadians(0))
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
