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

    HardwareMap hardwareMap;
    public SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


    public AutoTrajectories (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }


    Pose2d startPose = new Pose2d(StartPoseXBackdrop, StartPoseYBackdrop *S, StartPoseHeadingBackdrop *S+A);

    // static trajectories - might have to make these public
    Trajectory PreloadDrive1 = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(new Pose2d(PreloadPose1X,PreloadPose1Y*S, PreloadPose1Heading*S+A))
            .build();
    Trajectory PreloadDrive2 = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(new Pose2d(PreloadPose2X,PreloadPose2Y*S, PreloadPose2Heading*S+A))
            .build();
    Trajectory PreloadDrive3 = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(new Pose2d(PreloadPose3X,PreloadPose3Y*S,PreloadPose3Heading*S+A))
            .build();

    /* //shouldn't need to do this if drive is public :)
    public Pose2d getPoseEstimate(){
        return drive.getPoseEstimate();
    }
     */

    // might have to make multiple of these maybe
    public void driveIntoStackAfterBackPreload(Pose2d startTrajectory, double slowerVelocityIntoStack){ // generates a live trajectory
        Trajectory driveIntoStackAfterBackPreload = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(34, -33), Math.toRadians(-110)) // end tangent affects path alot
                .splineToConstantHeading(new Vector2d(23, -36), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-20, -36, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-37, -36, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// slower portion of spline
                .build();
        drive.followTrajectoryAsync(driveIntoStackAfterBackPreload);
    }

}
