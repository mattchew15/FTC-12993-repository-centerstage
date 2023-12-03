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

    MiddleLaneY = -31,
            LaneOffset = 27,

    ParkX = 45,
            ParkMiddleY = -14,
            ParkWallY = -54;


    public static Pose2d poseEstimate;

    Pose2d startPoseBack = new Pose2d(12, -59 *S, Math.toRadians(180) *S+A); // don't need to global variable as much with this setup
    Pose2d startPoseFront = new Pose2d(-38, -59, Math.toRadians(180)); // don't need to global variable as much with this setup

    Pose2d preload1Pose = new Pose2d(PreloadPose1X,PreloadPose1Y*S, PreloadPose1Heading*S+A); // not necessary lol
    Pose2d preload2Pose = new Pose2d(PreloadPose2X,PreloadPose2Y*S, PreloadPose2Heading*S+A);
    Pose2d preload3Pose = new Pose2d(PreloadPose3X,PreloadPose3Y*S,PreloadPose3Heading*S+A);

    public double middleDepositPoseX = 26.5;
    Pose2d middleDepositPose = new Pose2d(middleDepositPoseX, MiddleLaneY, Math.toRadians(180));

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
                .splineToConstantHeading(new Vector2d(34, -33), Math.toRadians(-110)) // end tangent affects path alot
                .splineToConstantHeading(new Vector2d(23, MiddleLaneY), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneY, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneY, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();

        driveIntoStackAfterBackPreload2 = drive.trajectoryBuilder(PreloadDrive2.end())
                .splineToConstantHeading(new Vector2d(34, MiddleLaneY), Math.toRadians(180)) // end tangent affects path alot
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneY, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneY, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();

        driveIntoStackAfterBackPreload3 = drive.trajectoryBuilder(PreloadDrive3.end())
                .splineToConstantHeading(new Vector2d(30, MiddleLaneY), Math.toRadians(180)) // end tangent affects path alot
                // .splineToConstantHeading(new Vector2d(23, MiddleLaneY), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneY, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneY, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();

        PreloadDrive1Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-60, -18, Math.toRadians(178))) // hardest one closest to stack
                .build();

        PreloadDrive2Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-48, -18.5, Math.toRadians(178)))
                .build();

        PreloadDrive3Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-41, -18, Math.toRadians(178)))
                .build();


        AfterPreloadDrive1Front = drive.trajectoryBuilder(PreloadDrive1Front.end())
                .lineToLinearHeading(new Pose2d(-54, FrontPreloadY, Math.toRadians(180))) // hardest one closest to stack
                .build();
        AfterPreloadDrive2Front = drive.trajectoryBuilder(PreloadDrive2Front.end())
                .lineToLinearHeading(new Pose2d(-45, FrontPreloadY, Math.toRadians(180))) // hardest one closest to stack
                .build();
        AfterPreloadDrive3Front = drive.trajectoryBuilder(PreloadDrive3Front.end())
                .lineToLinearHeading(new Pose2d(-38, FrontPreloadY, Math.toRadians(180))) // hardest one closest to stack
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
                .lineToSplineHeading(new Pose2d(10, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-28, MiddleLaneY+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset))), Math.toRadians(180)) // end tangent affects path alot\
                .lineTo(new Vector2d(-35.5, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset))), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
        drive.followTrajectoryAsync(driveIntoStackStraightLIVE);
    }

    public void outtakeDriveMiddlePath(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double depositX, double depositY){ // generates a live trajectory

         outtakeDriveMiddlePathLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-10, depositY, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(10, depositY), Math.toRadians(0)) // end tangent of path
                .lineTo(new Vector2d(depositX, depositY),SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveMiddlePathLIVE);
    }
    public void outtakeDriveTurnEndPath(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double depositX, int trussMiddleStage){ // generates a live trajectory
        Trajectory outtakeDriveTurnEndPath = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-5, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(0, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(10, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(endAngle)))
                .lineToSplineHeading(new Pose2d(depositX, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(endAngle)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndPath);
    }

    // will be a wierd spline depending on end coordinate
    public void driveIntoStackStageFromMiddlePathStraightEnd(Pose2d startTrajectory, double slowerVelocityIntoStack){ // generates a live trajectory
        Trajectory outtakeDriveMiddlePath = drive.trajectoryBuilder(startTrajectory) // might break because of generating trajectory
                .lineToSplineHeading(new Pose2d(32, MiddleLaneY + 4, Math.toRadians(175)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(32, MiddleLaneY + 6), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, MiddleLaneY + LaneOffset), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(20, MiddleLaneY + LaneOffset, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-17, MiddleLaneY + LaneOffset, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-37, MiddleLaneY + LaneOffset, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveMiddlePath);
    }
    public void park(Pose2d startTrajectory, int wallOrMiddle){ // generates a live trajectory
        Trajectory park = drive.trajectoryBuilder(startTrajectory)
                .lineTo(new Vector2d(ParkX, wallOrMiddle == 1?ParkWallY:ParkMiddleY)) // hardest one closest to stack
                .build();
        drive.followTrajectoryAsync(park);
    }




}
