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
    MiddleLaneYIntake = -31,
            LaneOffset = 26.4,

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
            AfterPreloadDrive3Front, driveIntoStackStraightMIDDLE, outtakeDriveMiddlePathLIVE;

    public void init(HardwareMap hardwareMap, LinearOpMode opMode)
    {
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
                .build();

        PreloadDrive3Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-55, -32*S, Math.toRadians(180)*S))
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
    public void driveIntoStackStraight(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage){ // generates a live trajectory
        Trajectory driveIntoStackStraightLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(9, MiddleLaneYIntake*S + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset*S: LaneOffset*S)), Math.toRadians(180)*S))
                .splineToConstantHeading(new Vector2d(-25, MiddleLaneYIntake*S+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset*S: LaneOffset*S))), Math.toRadians(180)*S) // end tangent affects path alot\
                .lineToSplineHeading(new Pose2d(-28, MiddleLaneYIntake*S + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset*S: LaneOffset*S)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
        drive.followTrajectoryAsync(driveIntoStackStraightLIVE);
    }

    public void driveIntoStackStraightFromMiddleTurnDrive(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage){ // generates a live trajectory
        Trajectory driveIntoStackStraightLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(20, MiddleLaneYDeposit*S + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset*S: LaneOffset*S)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-28, MiddleLaneYIntake*S+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset*S: LaneOffset*S))), Math.toRadians(180)*S) // end tangent affects path alot\
                .lineToSplineHeading(new Pose2d(-28, MiddleLaneYIntake*S + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset*S: LaneOffset*S)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
        drive.followTrajectoryAsync(driveIntoStackStraightLIVE);
    }

    public void goBackToStack(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage){ // generates a live trajectory
        Trajectory driveBackIntoStack = drive.trajectoryBuilder(startTrajectory)
                .lineToLinearHeading(new Pose2d( -28,MiddleLaneYIntake*S + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset*S: LaneOffset*S)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectoryAsync(driveBackIntoStack);
    }

    public void outtakeDriveMiddlePath(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double depositX, double depositY){ // generates a live trajectory

         outtakeDriveMiddlePathLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-10, depositY*S, Math.toRadians(180)*S))
                .splineToConstantHeading(new Vector2d(22.5, depositY*S), Math.toRadians(0)*S) // end tangent of path
                .lineTo(new Vector2d(depositX, depositY*S),SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveMiddlePathLIVE);
    }
    public void outtakeDriveTurnEndPath(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double depositX, int trussMiddleStage){ // generates a live trajectory
        Trajectory outtakeDriveTurnEndPath = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-5, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(180)*S))
                .splineToSplineHeading(new Pose2d(0, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(180)*S), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(endAngle)*S))
                .lineToSplineHeading(new Pose2d(depositX, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(endAngle)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndPath);
    }

    public void outtakeDriveTurnEndPathChangeX(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double depositX, double depositYOffset, int trussMiddleStage){
        Trajectory outtakeDriveTurnEndPathChangeX = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-8, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(180)*S))
                .splineToSplineHeading(new Pose2d(20, (MiddleLaneYDeposit - (depositYOffset) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(endAngle)*S),Math.toRadians(0)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(25, (MiddleLaneYDeposit - (depositYOffset) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(endAngle)*S), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(depositX, (MiddleLaneYDeposit - (depositYOffset) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(endAngle)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndPathChangeX);
    }

    public void outtakeDriveFromStraightTurnEndStageV2(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double yOffset){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory)

                .lineToSplineHeading(new Pose2d(-8, (-36+ LaneOffset)*S, Math.toRadians(180)*S))
                .splineToSplineHeading(new Pose2d(10, (-36+ LaneOffset)*S, Math.toRadians(endAngle)*S),Math.toRadians(0)*S)
                .lineToSplineHeading(new Pose2d(15, (-36 + LaneOffset)*S, Math.toRadians(endAngle)*S))
                .splineToConstantHeading(new Vector2d(32, (-36 - yOffset + LaneOffset)*S),Math.toRadians(-20)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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

    public void driveIntoStackAngledAfterAngledOuttake(Pose2d startTrajectory, double slowerVelocityIntoBackdrop){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory)

                .splineToConstantHeading(new Vector2d(15, -36*S),Math.toRadians(180)*S)
                .lineToSplineHeading(new Pose2d(10, -36*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-15, -36*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-18, -36*S, Math.toRadians(160)*S))
                .splineToConstantHeading(new Vector2d(-36, -32*S),Math.toRadians(160)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))


                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
    }

    public void outtakeDriveFromAngleTurnEndStage(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory)

                .splineToConstantHeading(new Vector2d(-18, -36),Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(0, -36, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(20, -36, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(24, -36, Math.toRadians(endAngle)))
                .splineToConstantHeading(new Vector2d(36, -32),Math.toRadians(20), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))


                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
    }

    // will be a wierd spline depending on end coordinate
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
    public void park(Pose2d startTrajectory, int wallOrMiddle){ // generates a live trajectory
        Trajectory park = drive.trajectoryBuilder(startTrajectory)
                .lineTo(new Vector2d(ParkX, wallOrMiddle == 1?ParkWallY*S:ParkMiddleY*S)) // hardest one closest to stack
                .build();
        drive.followTrajectoryAsync(park);
    }




}
