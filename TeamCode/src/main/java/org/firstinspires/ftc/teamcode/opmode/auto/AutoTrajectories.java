package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoGlobals.*;

public class AutoTrajectories {

    public SampleMecanumDrive drive; // if it throw null pointer exception make a hardware object and do this. in constructor

    public AutoTrajectories (HardwareMap hardwareMap){
        drive = new SampleMecanumDrive(hardwareMap);
    }


    Pose2d startPoseBack = new Pose2d(12, -59 *S, Math.toRadians(180) *S+A); // don't need to global variable as much with this setup
    Pose2d startPoseFront = new Pose2d(-38, -59 *S, Math.toRadians(180) *S+A); // don't need to global variable as much with this setup

    Pose2d preload1Pose = new Pose2d(PreloadPose1X,PreloadPose1Y*S, PreloadPose1Heading*S+A); // not necessary lol
    Pose2d preload2Pose = new Pose2d(PreloadPose2X,PreloadPose2Y*S, PreloadPose2Heading*S+A);
    Pose2d preload3Pose = new Pose2d(PreloadPose3X,PreloadPose3Y*S,PreloadPose3Heading*S+A);


    // static trajectories - might have to make these public
    Trajectory PreloadDrive1 = drive.trajectoryBuilder(startPoseBack)
            .lineToLinearHeading(preload1Pose)
            .build();
    Trajectory PreloadDrive2 = drive.trajectoryBuilder(startPoseBack)
            .lineToLinearHeading(preload2Pose)
            .build();
    Trajectory PreloadDrive3 = drive.trajectoryBuilder(startPoseBack)
            .lineToLinearHeading(preload3Pose)
            .build();

    Trajectory driveIntoStackAfterBackPreload1 = drive.trajectoryBuilder(PreloadDrive1.end())
            .splineToConstantHeading(new Vector2d(34, -33), Math.toRadians(-110)) // end tangent affects path alot
            .splineToConstantHeading(new Vector2d(23, MiddleLaneY), Math.toRadians(180))
            .lineToSplineHeading(new Pose2d(-20, MiddleLaneY, Math.toRadians(180)))
            .lineToSplineHeading(new Pose2d(-37, MiddleLaneY, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
            .build();

    Trajectory driveIntoStackAfterBackPreload2 = drive.trajectoryBuilder(PreloadDrive2.end())
            .splineToConstantHeading(new Vector2d(34, MiddleLaneY), Math.toRadians(180)) // end tangent affects path alot
            .lineToSplineHeading(new Pose2d(-20, MiddleLaneY, Math.toRadians(180)))
            .lineToSplineHeading(new Pose2d(-37, MiddleLaneY, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
            .build();

    Trajectory driveIntoStackAfterBackPreload3 = drive.trajectoryBuilder(PreloadDrive3.end())
            .splineToConstantHeading(new Vector2d(30, MiddleLaneY), Math.toRadians(180)) // end tangent affects path alot
            // .splineToConstantHeading(new Vector2d(23, MiddleLaneY), Math.toRadians(180))
            .lineToSplineHeading(new Pose2d(-20, MiddleLaneY, Math.toRadians(180)))
            .lineToSplineHeading(new Pose2d(-37, MiddleLaneY, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
            .build();


    Trajectory PreloadDrive1Front = drive.trajectoryBuilder(startPoseFront)
            .lineToLinearHeading(new Pose2d(-56, -24, Math.toRadians(-180))) // hardest one closest to stack
            .build();

    Trajectory PreloadDrive2Front = drive.trajectoryBuilder(startPoseFront)
            .lineToLinearHeading(new Pose2d(-47, -24, Math.toRadians(-180)))
            .build();

    Trajectory PreloadDrive3Front = drive.trajectoryBuilder(startPoseFront)
            .lineToLinearHeading(new Pose2d(-38, -24, Math.toRadians(-180)))
            .build();


    Trajectory AfterPreloadDrive1Front = drive.trajectoryBuilder(PreloadDrive1Front.end())
            .lineToLinearHeading(new Pose2d(-54, -37, Math.toRadians(-180))) // hardest one closest to stack
            .build();
    Trajectory AfterPreloadDrive2Front = drive.trajectoryBuilder(PreloadDrive2Front.end())
            .lineToLinearHeading(new Pose2d(-45, -37, Math.toRadians(-180))) // hardest one closest to stack
            .build();
    Trajectory AfterPreloadDrive3Front = drive.trajectoryBuilder(PreloadDrive3Front.end())
            .lineToLinearHeading(new Pose2d(-38, -37, Math.toRadians(-180))) // hardest one closest to stack
            .build();

    


    /* //shouldn't need to do this if drive is public :)
    public Pose2d getPoseEstimate(){
        return drive.getPoseEstimate();
    }
     */

    // live trajectories

    // might have to make multiple of these maybe
    public void driveIntoStackStraight(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage){ // generates a live trajectory
        Trajectory driveIntoStack = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-20, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-30, MiddleLaneY+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset))), Math.toRadians(180)) // end tangent affects path alot\
                .lineTo(new Vector2d(-37, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset))), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
        drive.followTrajectoryAsync(driveIntoStack);
    }

    public void outtakeDriveMiddlePath(Pose2d startTrajectory, double slowerVelocityIntoBackdrop){ // generates a live trajectory
        Trajectory outtakeDriveMiddlePath = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(0, MiddleLaneY, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(27, MiddleLaneY), Math.toRadians(0)) // end tangent of path
                .lineTo(new Vector2d(32, MiddleLaneY),SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveMiddlePath);
    }
    public void outtakeDriveTurnEndPath(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, int trussMiddleStage){ // generates a live trajectory
        Trajectory outtakeDriveTurnEndPath = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(0, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(5, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(180)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(27, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(endAngle)))
                .lineToSplineHeading(new Pose2d(36, MiddleLaneY + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffset: LaneOffset)), Math.toRadians(endAngle)), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndPath);
    }

    // will be a wierd spline depending on end coordinate
    public void driveIntoStackStageFromMiddlePathStraightEnd(Pose2d startTrajectory, double slowerVelocityIntoStack){ // generates a live trajectory
        Trajectory outtakeDriveMiddlePath = drive.trajectoryBuilder(startTrajectory) // might break because of generating trajectory
                .lineToSplineHeading(new Pose2d(32, MiddleLaneY + 4, Math.toRadians(175)))
                .splineToConstantHeading(new Vector2d(32, MiddleLaneY + 6), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(25, MiddleLaneY + LaneOffset), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(20, MiddleLaneY + LaneOffset, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-30, MiddleLaneY + LaneOffset, Math.toRadians(180)))
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
