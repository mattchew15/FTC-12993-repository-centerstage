package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveThread;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
//import static org.firstinspires.ftc.teamcode.opmode.auto.AutoGlobals.*;

public class AutoTrajectories {

    public SampleMecanumDriveThread drive; // if it throw null pointer exception make a hardware object and do this. in constructor

    public AutoTrajectories (){
    }

    public boolean preExtendSlides;
    public boolean dropPurple;
    public boolean extendSlidesAroundTruss;
    public boolean extendSlidesAroundStage;
    public boolean extendOuttakeSlidesAroundTruss;

    public static double

    xPosition,
    yPosition,
    headingPosition,

    SlowerVelocityConstraintIntake = 15,
    SlowerVelocityConstraintDeposit = 15,
    SlowerVelocityConstraintDepositFaster = 25,

    StartPoseYBackdrop = -59,
    StartPoseXBackdrop = 12,
    StartPoseHeadingBackdrop = Math.toRadians(180),

    FrontPreloadY = -37,

    PreloadPose1X = 35,
            PreloadPose1Y = -32,
            PreloadPose1Heading = Math.toRadians(180),

    PreloadPose2X = 35,
            PreloadPose2Y = -35,
            PreloadPose2Heading = Math.toRadians(175),

    PreloadPose3X = 35,
            PreloadPose3Y = -38,
            PreloadPose3Heading = Math.toRadians(170),

    MiddleLaneYDeposit = -32,
    MiddleLaneYIntake = -30.2,
            LaneOffset = 23,
            LaneOffsetTruss = 22.3,

    ParkX = 45,
            ParkMiddleY = -14,
            ParkWallY = -54,

    TurnLeftFrontPreload = -176,
    TurnRightFrontPreload = 176;


    public static Pose2d poseEstimate;

    Pose2d startPoseBack = new Pose2d(9.9, -59 *S, Math.toRadians(180) *S); // don't need to global variable as much with this setup
    Pose2d startPoseFront = new Pose2d(-38, -59*S, Math.toRadians(180)*S);

    Pose2d preload1Pose = new Pose2d(PreloadPose1X,PreloadPose1Y*S, PreloadPose1Heading*S); // not necessary lol
    Pose2d preload2Pose = new Pose2d(PreloadPose2X,PreloadPose2Y*S, PreloadPose2Heading*S);
    Pose2d preload3Pose = new Pose2d(PreloadPose3X,PreloadPose3Y*S,PreloadPose3Heading*S);

    public double middleDepositPoseX = 26.5;
    Pose2d middleDepositPose = new Pose2d(middleDepositPoseX, MiddleLaneYDeposit*S, Math.toRadians(180)*S);

    Trajectory PreloadDrive1, PreloadDrive2, PreloadDrive3, driveIntoStackAfterBackPreload1,
            driveIntoStackAfterBackPreload2, driveIntoStackAfterBackPreload3, PreloadDrive1Front,
            PreloadDrive2Front, PreloadDrive3Front, AfterPreloadDrive1Front, AfterPreloadDrive2Front,
            AfterPreloadDrive3Front, driveIntoStackStraightMIDDLE, outtakeDriveMiddlePathLIVE, driveIntoStackStageFromMiddlePathStraightEndV2,
    firstDriveThroughTrussAfterPurple2, PreloadDrive2FrontStage, PreloadDrive1FrontStage,PreloadDrive3FrontStage
            ,firstDriveThroughStageAfterPurple2;

    public void init(HardwareMap hardwareMap, LinearOpMode opMode)
    {
        preExtendSlides = false;
        dropPurple = false;
        drive = new SampleMecanumDriveThread(hardwareMap);
        drive.startImuThread(opMode);

        // static trajectories - might have to make these public
         PreloadDrive1 = drive.trajectoryBuilder(startPoseBack)
                .lineToLinearHeading(new Pose2d(33,-24*S, Math.toRadians(180)*S))
                .build();
         PreloadDrive2 = drive.trajectoryBuilder(startPoseBack)
                .lineToLinearHeading(new Pose2d(35.5,-24.4*S, Math.toRadians(171)*S))
                .build();
         PreloadDrive3 = drive.trajectoryBuilder(startPoseBack)
                .lineToLinearHeading(new Pose2d(38,-27.7*S, Math.toRadians(159)*S))
                .build();


         // pregenerated trajectory
        driveIntoStackStageFromMiddlePathStraightEndV2 = drive.trajectoryBuilder(new Pose2d(26,MiddleLaneYDeposit, Math.toRadians(180))) // might break because of generating trajectory
                .lineToSplineHeading(new Pose2d(24.1, (MiddleLaneYDeposit + 4)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(20, (MiddleLaneYDeposit + 10)*S), Math.toRadians(110)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(6, (MiddleLaneYDeposit + LaneOffsetTruss)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(5, (MiddleLaneYDeposit + LaneOffsetTruss)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-20, (MiddleLaneYDeposit + LaneOffsetTruss)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-28, (MiddleLaneYDeposit + LaneOffsetTruss)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();

         driveIntoStackAfterBackPreload1 = drive.trajectoryBuilder(PreloadDrive1.end())
                .splineToConstantHeading(new Vector2d(34, -33*S), Math.toRadians(-110)*S,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // end tangent affects path alot
                .splineToConstantHeading(new Vector2d(23, MiddleLaneYIntake*S), Math.toRadians(180)*S,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneYIntake*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneYIntake*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();

        driveIntoStackAfterBackPreload2 = drive.trajectoryBuilder(PreloadDrive2.end())
                .splineToConstantHeading(new Vector2d(34, MiddleLaneYDeposit*S), Math.toRadians(180)*S,SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // end tangent affects path alot
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneYDeposit*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneYDeposit*S, Math.toRadians(175)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();

        driveIntoStackAfterBackPreload3 = drive.trajectoryBuilder(PreloadDrive3.end())
                .splineToConstantHeading(new Vector2d(30, MiddleLaneYDeposit*S), Math.toRadians(180)*S,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // end tangent affects path alot
                // .splineToConstantHeading(new Vector2d(23, MiddleLaneYIntake), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneYDeposit*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneYDeposit*S, Math.toRadians(175)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();

        PreloadDrive1Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-58, -32*S, Math.toRadians(180)*S)) // hardest one closest to stack
                .build();

        PreloadDrive2Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-50, -32*S, Math.toRadians(180)*S))
                .addSpatialMarker(new Vector2d(-50, 25), () -> {
                    preExtendSlides = true;
                })
                .addSpatialMarker(new Vector2d(-50, -31), () -> {
                    dropPurple = true;
                })
                .build();

        PreloadDrive3Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-55, -32*S, Math.toRadians(180)*S))
                .build();

        PreloadDrive2FrontStage = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-50, -9*S, Math.toRadians(180)*S))
                .addSpatialMarker(new Vector2d(-50, -16), () -> {
                    preExtendSlides = true;
                })
                .addSpatialMarker(new Vector2d(-50, -15), () -> {
                    dropPurple = true;
                })
                .build();

        firstDriveThroughStageAfterPurple2 = drive.trajectoryBuilder(PreloadDrive2FrontStage.end(), true)

                .lineToSplineHeading(new Pose2d(20, -9, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d( 4, -9), () -> {
                    extendSlidesAroundTruss = true;
                })
                .splineToConstantHeading(new Vector2d(36, (-9 - 12)), Math.toRadians(-42), SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();


        firstDriveThroughTrussAfterPurple2 = drive.trajectoryBuilder(PreloadDrive2Front.end(), true)

                .splineToConstantHeading(new Vector2d(-41, (MiddleLaneYDeposit + 4 - LaneOffsetTruss)), Math.toRadians(-35), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-18, (MiddleLaneYDeposit - LaneOffsetTruss)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-5, MiddleLaneYDeposit - LaneOffsetTruss, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addSpatialMarker(new Vector2d( 5, MiddleLaneYDeposit - LaneOffsetTruss), () -> {
                    extendSlidesAroundTruss = true;
                })
                .splineToConstantHeading(new Vector2d(32.1, (MiddleLaneYDeposit + 14.2 -LaneOffsetTruss)), Math.toRadians(55), SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        AfterPreloadDrive1Front = drive.trajectoryBuilder(PreloadDrive1Front.end())
                .lineToLinearHeading(new Pose2d(-54, FrontPreloadY*S, Math.toRadians(180)*S)) // hardest one closest to stack
                .build();
        AfterPreloadDrive2Front = drive.trajectoryBuilder(PreloadDrive2Front.end())
                .lineToLinearHeading(new Pose2d(-45, FrontPreloadY*S, Math.toRadians(180)*S)) // hardest one closest to stack
                .build();
        AfterPreloadDrive3Front = drive.trajectoryBuilder(PreloadDrive3Front.end())
                .lineToLinearHeading(new Pose2d(-38, FrontPreloadY*S, Math.toRadians(180)*S)) // hardest one closest to stack
                .build();


        /*
        driveIntoStackStraightMIDDLE = drive.trajectoryBuilder(middleDepositPose)
                .lineToSplineHeading(new Pose2d(10, MiddleLaneY, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-16, MiddleLaneY), Math.toRadians(180)) // end tangent affects path alot\
                .lineTo(new Vector2d(-36, MiddleLaneY), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();

         */

    }


    /* //shouldn't need to do this if drive is public :)
    public Pose2d getPoseEstimate(){
        return drive.getPoseEstimate();
    }
     */

    // live trajectories

    // might have to make multiple of these maybe
    public void driveIntoStackStraight(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage, double positiveYOffset){ // generates a live trajectory
        Trajectory driveIntoStackStraightLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(9, MiddleLaneYIntake*S +positiveYOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)), Math.toRadians(180)*S))
                .splineToConstantHeading(new Vector2d(-25, MiddleLaneYIntake*S +positiveYOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), Math.toRadians(180)*S) // end tangent affects path alot\
                .lineToSplineHeading(new Pose2d(-28, MiddleLaneYIntake*S +positiveYOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
        drive.followTrajectoryAsync(driveIntoStackStraightLIVE);
    }
    public Trajectory driveIntoStackStraightTrajectory(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage, double positiveYOffset){ // generates a live trajectory
        return drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(9, MiddleLaneYIntake*S +positiveYOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)), Math.toRadians(180)*S))
                .splineToConstantHeading(new Vector2d(-25, MiddleLaneYIntake*S +positiveYOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), Math.toRadians(180)*S) // end tangent affects path alot\
                .lineToSplineHeading(new Pose2d(-28.5, MiddleLaneYIntake*S +positiveYOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
    }

    public void driveIntoStackStraightFromMiddleTurnDrive(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage){ // generates a live trajectory
        Trajectory driveIntoStackStraightLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(20, MiddleLaneYDeposit*S + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-28, MiddleLaneYIntake*S+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), Math.toRadians(180)*S) // end tangent affects path alot\
                .lineToSplineHeading(new Pose2d(-28, MiddleLaneYIntake*S + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
        drive.followTrajectoryAsync(driveIntoStackStraightLIVE);
    }

    public void goBackToStack(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage){ // generates a live trajectory
        Trajectory driveBackIntoStack = drive.trajectoryBuilder(startTrajectory)
                .lineToLinearHeading(new Pose2d( -28,MiddleLaneYIntake*S + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectoryAsync(driveBackIntoStack);
    }

    public void outtakeDriveMiddlePath(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double depositX, double depositY){ // generates a live trajectory

         outtakeDriveMiddlePathLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-10, depositY*S, Math.toRadians(180)*S))
                .splineToConstantHeading(new Vector2d(22, depositY*S), Math.toRadians(0)*S) // end tangent of path
                .lineTo(new Vector2d(depositX, depositY*S),SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveMiddlePathLIVE);
    }
    public Trajectory outtakeDriveMiddlePathTrajectory(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double depositX, double depositY){
        return drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-10, depositY*S, Math.toRadians(180)*S))
                .splineToConstantHeading(new Vector2d(22, depositY*S), Math.toRadians(0)*S) // end tangent of path
                .lineTo(new Vector2d(depositX, depositY*S),SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
    }

    public void outtakeDriveTurnEndPath(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double depositX, int trussMiddleStage){ // generates a live trajectory
        Trajectory outtakeDriveTurnEndPath = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-5, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S, Math.toRadians(180)*S))
                .splineToSplineHeading(new Pose2d(0, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S, Math.toRadians(180)*S), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S, Math.toRadians(endAngle)*S))
                .lineToSplineHeading(new Pose2d(depositX, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S, Math.toRadians(endAngle)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndPath);
    }

    public void outtakeDriveTurnEndPathChangeX(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double depositX, double depositYOffset, int trussMiddleStage){
        Trajectory outtakeDriveTurnEndPathChangeX = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-8, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S, Math.toRadians(180)*S))
                .splineToSplineHeading(new Pose2d(20, (MiddleLaneYDeposit - (depositYOffset) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S, Math.toRadians(endAngle)*S),Math.toRadians(0)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(25, (MiddleLaneYDeposit - (depositYOffset) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S, Math.toRadians(endAngle)*S), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(depositX, (MiddleLaneYDeposit - (depositYOffset) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S, Math.toRadians(endAngle)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndPathChangeX);
    }

    // this is the old spline/turn path
    public void outtakeDriveFromStraightTurnEndStageV2(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double yOffset){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory)

                .lineToSplineHeading(new Pose2d(-8, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(180)*S))
                .splineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(endAngle)*S),Math.toRadians(0)*S)
                .lineToSplineHeading(new Pose2d(15, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(endAngle)*S))
                .splineToConstantHeading(new Vector2d(32, (MiddleLaneYDeposit - yOffset + LaneOffset)*S),Math.toRadians(-20)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
    }
    public Trajectory outtakeDriveFromStraightTUrnEndStageV2Trajectory(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double yOffset){
        return drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-8, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(180)*S))
                .splineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(endAngle)*S),Math.toRadians(0)*S)
                .lineToSplineHeading(new Pose2d(15, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(endAngle)*S))
                .splineToConstantHeading(new Vector2d(32, (MiddleLaneYDeposit - yOffset + LaneOffset)*S),Math.toRadians(-20)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }
    public Trajectory outtakeDriveFromStraightTUrnEndStageV2TrussTrajectory(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double yOffset){
        return drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-8, (MiddleLaneYDeposit- LaneOffsetTruss)*S, Math.toRadians(-180)*S))
                .splineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit- LaneOffsetTruss)*S, Math.toRadians(endAngle)*S),Math.toRadians(0)*S)
                .lineToSplineHeading(new Pose2d(15, (MiddleLaneYDeposit - LaneOffsetTruss)*S, Math.toRadians(endAngle)*S))
                .splineToConstantHeading(new Vector2d(32, (MiddleLaneYDeposit + yOffset - LaneOffsetTruss)*S),Math.toRadians(20)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }

    public void outtakeDriveFromStraightTurnEndStageV3(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double yOffset){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory, true)

                .splineTo(new Vector2d(-18,(MiddleLaneYDeposit+ LaneOffset)*S), Math.toRadians(0)*S)
                .lineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(0)*S))
                .splineTo(new Vector2d(32,(MiddleLaneYDeposit+ LaneOffset + yOffset)*S), Math.toRadians(endAngle)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
    }

    public void driveIntoStackStraightAfterAngledOuttakeSTAGE(Pose2d startTrajectory, double slowerVelocityIntoBackdrop){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory)

                .splineToConstantHeading(new Vector2d(15, (MiddleLaneYDeposit + LaneOffset)*S),Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-36, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
    }

    public void driveIntoStackAngledAfterAngledOuttake(Pose2d startTrajectory, double slowerVelocityIntoStack, double yOffset, double endAngle){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory)

                .splineTo(new Vector2d(15,(MiddleLaneYDeposit+ LaneOffset)*S), Math.toRadians(180)*S)
                .lineToSplineHeading(new Pose2d(-15, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(180)*S))
                .splineTo(new Vector2d(-36,(MiddleLaneYDeposit+ LaneOffset + yOffset)*S), Math.toRadians(endAngle)*S,SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // roughly 160

                /*
                .splineToConstantHeading(new Vector2d(15, -36*S),Math.toRadians(180)*S)
                .lineToSplineHeading(new Pose2d(10, -36*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-15, -36*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-18, -36*S, Math.toRadians(160)*S))
                .splineToConstantHeading(new Vector2d(-36, -32*S),Math.toRadians(160)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 */

                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
    }
    public Trajectory driveIntoStackAngledAfterAngledOuttakeTrajectory(Pose2d startTrajectory, double slowerVelocityIntoStack, double yOffset, double endAngle, int trussMiddleStage, double positiveYDriftOffset){
        return drive.trajectoryBuilder(startTrajectory)

                //.splineTo(new Vector2d(15,(MiddleLaneYDeposit+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)))*S), Math.toRadians(180)*S,SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                 //       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(9, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)), Math.toRadians(180)*S),SampleMecanumDrive.getVelocityConstraint(trussMiddleStage == 1? 35: DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-15, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), Math.toRadians(180)*S) // end tangent affects path alot\
               // .lineToSplineHeading(new Pose2d(-15, (MiddleLaneYDeposit+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)))*S, Math.toRadians(180)*S))
                .addSpatialMarker(new Vector2d(-24.5, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), () -> {
                    extendSlidesAroundTruss = true;
                })
                .addSpatialMarker(new Vector2d(-11, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), () -> {
                    extendSlidesAroundStage = true;
                })
                .splineTo(new Vector2d(-36,(MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)) + yOffset)*S), Math.toRadians(endAngle)*S,SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // roughly 160
                /*
                .splineToConstantHeading(new Vector2d(15, -36*S),Math.toRadians(180)*S)
                .lineToSplineHeading(new Pose2d(10, -36*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-15, -36*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-18, -36*S, Math.toRadians(160)*S))
                .splineToConstantHeading(new Vector2d(-36, -32*S),Math.toRadians(160)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 */
                .build();
    }

    public void outtakeDriveFromAngleTurnEnd(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double yOffset){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory, true)

                .splineTo(new Vector2d(-18,(MiddleLaneYDeposit+ LaneOffset)*S), Math.toRadians(0)*S)
                .lineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(0)*S))
                .splineTo(new Vector2d(32,(MiddleLaneYDeposit+ LaneOffset + yOffset)*S), Math.toRadians(endAngle)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
    }

    public Trajectory outtakeDriveFromAngleTurnEndTrajectory(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double yOffset, int trussMiddleStage){
        return drive.trajectoryBuilder(startTrajectory, true)

                .splineTo(new Vector2d(-18,(MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S), Math.toRadians(0)*S)
                .addSpatialMarker(new Vector2d( 5, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S), () -> {
                    extendSlidesAroundTruss = true;
                })
                .lineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S, Math.toRadians(180)*S))
                .splineTo(new Vector2d(32,(MiddleLaneYDeposit+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)) + yOffset)*S), Math.toRadians(endAngle)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(32,(MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffset)))*S),Math.toRadians(-20)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
    }

    // v1 - for other purple pixel cases
    public void driveIntoStackStageFromMiddlePathStraightEnd(Pose2d startTrajectory, double slowerVelocityIntoStack, double slowerVelocityStart){ // generates a live trajectory
        Trajectory outtakeDriveMiddlePath = drive.trajectoryBuilder(startTrajectory) // might break because of generating trajectory
                .lineToSplineHeading(new Pose2d(32, (MiddleLaneYDeposit + 4)*S, Math.toRadians(175)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityStart, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(32, (MiddleLaneYDeposit + 6)*S), Math.toRadians(90)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, (MiddleLaneYDeposit + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityStart, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(20, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-27, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-37, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveMiddlePath);
    }

    //TODO run this in init and then just deal with the error
    public void driveIntoStackStageFromMiddlePathStraightEndV2(Pose2d startTrajectory, double slowerVelocityIntoStack, double slowerVelocityStart){ // generates a live trajectory
        Trajectory outtakeDriveMiddlePath = drive.trajectoryBuilder(startTrajectory) // might break because of generating trajectory
                .lineToSplineHeading(new Pose2d(24.1, (MiddleLaneYDeposit + 4)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityStart, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(20, (MiddleLaneYDeposit + 10)*S), Math.toRadians(110)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityStart, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(6, (MiddleLaneYDeposit + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityStart, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(5, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-20, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-28, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveMiddlePath);
    }

    public void park(Pose2d startTrajectory, int wallOrMiddle){ // generates a live trajectory
        Trajectory park = drive.trajectoryBuilder(startTrajectory)
                .lineTo(new Vector2d(ParkX, wallOrMiddle == 1?ParkWallY*S:ParkMiddleY*S)) // hardest one closest to stack
                .build();
        drive.followTrajectoryAsync(park);
    }
    public Trajectory parkTrajectory(Pose2d startTrajectory, int wallOrMiddle){
        return drive.trajectoryBuilder(startTrajectory)
                .lineTo(new Vector2d(ParkX, wallOrMiddle == 1?ParkWallY*S:ParkMiddleY*S)) // hardest one closest to stack
                .build();
    }




}
