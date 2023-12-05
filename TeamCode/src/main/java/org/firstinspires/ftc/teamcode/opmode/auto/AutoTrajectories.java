package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
//import static org.firstinspires.ftc.teamcode.opmode.auto.AutoGlobals.*;

public class AutoTrajectories {

    public SampleMecanumDrive drive; // if it throw null pointer exception make a hardware object and do this. in constructor

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
    MiddleLaneYIntake = -28.7,
            LaneOffset = 27.3,

    ParkX = 45,
            ParkMiddleY = -14,
            ParkWallY = -54;


    public static Pose2d poseEstimate;

    Pose2d startPoseBack = new Pose2d(12, -59 *S, Math.toRadians(180) *S); // don't need to global variable as much with this setup
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

    public void init(HardwareMap hardwareMap)
    {
        drive = new SampleMecanumDrive(hardwareMap);

        // static trajectories - might have to make these public
         PreloadDrive1 = drive.trajectoryBuilder(startPoseBack)
                .lineToLinearHeading(preload1Pose)
                .build();
         PreloadDrive2 = drive.trajectoryBuilder(startPoseBack)
                .lineToLinearHeading(preload2Pose)
                .build();
         PreloadDrive3 = drive.trajectoryBuilder(startPoseBack)
                .lineToLinearHeading(preload3Pose)
                .build();

         driveIntoStackAfterBackPreload1 = drive.trajectoryBuilder(PreloadDrive1.end())
                .splineToConstantHeading(new Vector2d(34, -33*S), Math.toRadians(-110)*S) // end tangent affects path alot
                .splineToConstantHeading(new Vector2d(23, MiddleLaneYIntake*S), Math.toRadians(180)*S)
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneYIntake*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneYIntake*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();

        driveIntoStackAfterBackPreload2 = drive.trajectoryBuilder(PreloadDrive2.end())
                .splineToConstantHeading(new Vector2d(34, MiddleLaneYIntake*S), Math.toRadians(180)*S) // end tangent affects path alot
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneYIntake*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneYIntake*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();

        driveIntoStackAfterBackPreload3 = drive.trajectoryBuilder(PreloadDrive3.end())
                .splineToConstantHeading(new Vector2d(30, MiddleLaneYIntake*S), Math.toRadians(180)*S) // end tangent affects path alot
                // .splineToConstantHeading(new Vector2d(23, MiddleLaneYIntake), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneYIntake*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneYIntake*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();

        PreloadDrive1Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-62, -19*S, Math.toRadians(179)*S)) // hardest one closest to stack
                .build();

        PreloadDrive2Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-50, -19*S, Math.toRadians(178)*S))
                .build();

        PreloadDrive3Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-40, -19*S, Math.toRadians(178)*S))
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
                .lineToSplineHeading(new Pose2d(10, MiddleLaneYIntake + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)*S))
                .splineToConstantHeading(new Vector2d(-28, MiddleLaneYIntake+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset))), Math.toRadians(180)*S) // end tangent affects path alot\
                .lineToSplineHeading(new Pose2d(-36.7, MiddleLaneYIntake + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
        drive.followTrajectoryAsync(driveIntoStackStraightLIVE);
    }

    public void driveIntoStackStraightFromMiddleTurnDrive(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage){ // generates a live trajectory
        Trajectory driveIntoStackStraightLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(20, MiddleLaneYIntake + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-28, MiddleLaneYIntake+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset))), Math.toRadians(180)*S) // end tangent affects path alot\
                .lineToSplineHeading(new Pose2d(-36.7, MiddleLaneYIntake + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
        drive.followTrajectoryAsync(driveIntoStackStraightLIVE);
    }

    public void goBackToStack(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage){ // generates a live trajectory
        Trajectory driveBackIntoStack = drive.trajectoryBuilder(startTrajectory)
                .lineToLinearHeading(new Pose2d( -36.7,MiddleLaneYIntake + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectoryAsync(driveBackIntoStack);
    }

    public void outtakeDriveMiddlePath(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double depositX, double depositY){ // generates a live trajectory

         outtakeDriveMiddlePathLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-10, depositY*S, Math.toRadians(180)*S))
                .splineToConstantHeading(new Vector2d(10, depositY*S), Math.toRadians(0)*S) // end tangent of path
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
                .lineToSplineHeading(new Pose2d(-8, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(20, (MiddleLaneYDeposit - (depositYOffset*S) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(endAngle)),Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(25, (MiddleLaneYDeposit - (depositYOffset*S) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(endAngle)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(depositX, (MiddleLaneYDeposit - (depositYOffset*S) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)))*S, Math.toRadians(endAngle)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndPathChangeX);
    }

    public void outtakeDriveFromStraightTurnEndStageV2(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory)

                .lineToSplineHeading(new Pose2d(-8, -36, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(15, -36, Math.toRadians(endAngle)))
                .splineToConstantHeading(new Vector2d(31, -42),Math.toRadians(-20), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))


                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
    }

    public void driveIntoStackStraightAfterAngledOuttakeSTAGE(Pose2d startTrajectory, double slowerVelocityIntoBackdrop){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory)

                .splineToConstantHeading(new Vector2d(15, -36),Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(10, -36, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
    }

    public void driveIntoStackAngledAfterAngledOuttake(Pose2d startTrajectory, double slowerVelocityIntoBackdrop){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory)

                .splineToConstantHeading(new Vector2d(15, -36),Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(10, -36, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-15, -36, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-18, -36, Math.toRadians(160)))
                .splineToConstantHeading(new Vector2d(-36, -32),Math.toRadians(160), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
    public void driveIntoStackStageFromMiddlePathStraightEnd(Pose2d startTrajectory, double slowerVelocityIntoStack){ // generates a live trajectory
        Trajectory outtakeDriveMiddlePath = drive.trajectoryBuilder(startTrajectory) // might break because of generating trajectory
                .lineToSplineHeading(new Pose2d(32, MiddleLaneYIntake*S + 4, Math.toRadians(175)*S), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(32, MiddleLaneYIntake*S + 6), Math.toRadians(90)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, (MiddleLaneYIntake + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(20, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-27, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-37, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
