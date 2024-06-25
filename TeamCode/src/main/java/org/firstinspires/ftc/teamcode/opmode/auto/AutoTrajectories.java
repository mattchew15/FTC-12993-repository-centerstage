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
    public boolean extendSlidesForBackAutos;
    Pose2d trussSideStackCoordinate;

    public static double
    xPosition,
    yPosition,
    headingPosition;

    public double
        MiddleLaneYIntake,
        MiddleY = -32,
        TrussY = -53.5,
        StageY = -8,
        StageYIntake = -5.7;

    final public static double
// deposits and offsets
    // FrontPreloadY = -37,
//    MiddleY = -32,
//    TrussY = -52.8,
//    StageY = -8,
//    StageYIntake = -6,

    MiddleLaneYDeposit = -32,

    LaneOffset = 24,
    LaneOffsetTruss = 20.8,
    // value for pregenerated first drive from backdrop into stacks truss
    FirstTrussDriveOffset = 2.1,
    // used in 3 front cases and 3 back cases (6 total for pregenerated trajecotry)

// velocity constraints
    SlowerVelocityFirstTrussDriveToStacks = 28,
    // used for 6 cases (3 front and back) for pregenerated trajectories
    SlowerVelocityFirstStageDriveToStacks = 24,
    // pregenerated first stack truss angle
    SlowerVelocityConstraintIntake = 14,
    // these aren't needed really
    SlowerVelocityConstraintDeposit = 15,
    SlowerVelocityConstraintDepositFaster = 25,
// angle constraints
    StackTrussAngle1 = 154.5,

// park trajectories have been good
    ParkX = 45,
    ParkMiddleY = -14,
    ParkWallY = -54;

    public static Pose2d poseEstimate;
    Pose2d startPoseBack;
    Pose2d startPoseFront;

    public double middleDepositPoseX = 26.5;

    Trajectory PreloadDrive1, PreloadDrive2, PreloadDrive3, driveIntoStackAfterBackPreload1,
        driveIntoStackAfterBackPreload2, driveIntoStackAfterBackPreload3, PreloadDrive1Front,
        PreloadDrive2Front, PreloadDrive3Front, AfterPreloadDrive1Front, AfterPreloadDrive2Front,
        AfterPreloadDrive3Front, driveIntoStackStraightMIDDLE, outtakeDriveMiddlePathLIVE, driveIntoStackStageFromMiddlePathStraightEndV2, driveIntoStackStageFromMiddlePathStraightEndV1,
        firstDriveThroughTrussAfterPurple2,firstDriveThroughTrussAfterPurple1,firstDriveThroughTrussAfterPurple3,
        PreloadDrive2FrontStage, PreloadDrive1FrontStage,PreloadDrive3FrontStage
        ,firstDriveThroughStageAfterPurple2, firstDriveThroughStageAfterPurple3, firstDriveThroughStageAfterPurple1,
        driveIntoStacksAfterYellowStage2, driveIntoStacksAfterYellowStage1, driveIntoStacksAfterYellowStage3,PreloadDrive3FrontFirst,PreloadDrive3FrontSecond,
        driveIntoStacksAfterYellowTruss1,driveIntoStacksAfterYellowTruss2,driveIntoStacksAfterYellowTruss3,
        driveIntoStacksAfterBackTruss1,driveIntoStacksAfterBackTruss2,driveIntoStacksAfterBackTruss3, driveIntoStacksAfterBackStage2,driveIntoStacksAfterBackStage1,
        driveIntoStacksAfterBackStage3;

    public void init(HardwareMap hardwareMap, LinearOpMode opMode)
    {
        MiddleLaneYIntake = -30.85;
        preExtendSlides = false;
        dropPurple = false;
        drive = new SampleMecanumDriveThread(hardwareMap);
        drive.startImuThread(opMode);

        startPoseBack = new Pose2d(9.9, -59 *S, Math.toRadians(180) *S); // don't need to global variable as much with this setup
        startPoseFront = new Pose2d(-38, -59*S, Math.toRadians(180)*S);

        // static trajectories - might have to make these public
        double extendSLidesForBackYValue = -34;
        //Back preload drives
         PreloadDrive1 = drive.trajectoryBuilder(startPoseBack)
                .lineToLinearHeading(new Pose2d(36,-29*S, Math.toRadians(180)*S)
                        )
                 .addSpatialMarker(new Vector2d( 36, extendSLidesForBackYValue*S), () -> {
                     extendSlidesForBackAutos = true;
                 })
                .build();
         PreloadDrive2 = drive.trajectoryBuilder(startPoseBack)
                .lineToLinearHeading(new Pose2d(36,-26*S, Math.toRadians(174)*S))
                 .addSpatialMarker(new Vector2d( 36, extendSLidesForBackYValue*S), () -> {
                     extendSlidesForBackAutos = true;
                 })
                .build();
         PreloadDrive3 = drive.trajectoryBuilder(startPoseBack)
                .lineToLinearHeading(new Pose2d(36,-34*S, Math.toRadians(180)*S))
                 .addSpatialMarker(new Vector2d( 36, extendSLidesForBackYValue + 2*S), () -> {
                     extendSlidesForBackAutos = true;
                 })
                .build();
        driveIntoStackStageFromMiddlePathStraightEndV1 = drive.trajectoryBuilder(new Pose2d(26,MiddleLaneYDeposit, Math.toRadians(180))) // might break because of generating trajectory
                .lineToSplineHeading(new Pose2d(32, (MiddleLaneYDeposit + 4)*S, Math.toRadians(175)*S), SampleMecanumDrive.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(32, (MiddleLaneYDeposit + 6)*S), Math.toRadians(90)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, (MiddleLaneYDeposit + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(20, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-27, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-37, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // slower portion of spline
                .build();
        driveIntoStackStageFromMiddlePathStraightEndV2 = drive.trajectoryBuilder(new Pose2d(26,MiddleLaneYDeposit, Math.toRadians(180))) // might break because of generating trajectory
                .lineToSplineHeading(new Pose2d(24.1, (MiddleLaneYDeposit + 4)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(20, (MiddleLaneYDeposit + 10)*S), Math.toRadians(110)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(6, (MiddleLaneYDeposit + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(5, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-20, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-28, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                .lineToLinearHeading(new Pose2d(-60, -32.2*S, Math.toRadians(180)*S)) // hardest one closest to stack
                .addSpatialMarker(new Vector2d(-50, -32.5*S),() -> {
                    preExtendSlides = true;
                })
                .addSpatialMarker(new Vector2d(-60, -32*S), () -> {
                    dropPurple = true;
                })
                .build();
        PreloadDrive2Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-50, -32*S, Math.toRadians(180)*S))
                .addSpatialMarker(new Vector2d(-50, -32*S),() -> {
                    preExtendSlides = true;
                })
                .addSpatialMarker(new Vector2d(-50, -32*S), () -> {
                    dropPurple = true;
                })
                .build();
        PreloadDrive3Front = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-47, -31.7*S, Math.toRadians(180)*S))
                .addSpatialMarker(new Vector2d(-47, -32*S),() -> {
                    preExtendSlides = true;
                })
                .addSpatialMarker(new Vector2d(-47, -31.7*S), () -> {
                    dropPurple = true;
                })
                .build();
        PreloadDrive2FrontStage = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-48, -9.8*S, Math.toRadians(179)*S), SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addSpatialMarker(new Vector2d(-48, -10*S), () -> {
                    preExtendSlides = true;
                })
                .addSpatialMarker(new Vector2d(-48, -10*S), () -> {
                    dropPurple = true;
                })
                .build();
        PreloadDrive1FrontStage = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-60, -9*S, Math.toRadians(180)*S))
                .addSpatialMarker(new Vector2d(-60, -16*S), () -> {
                    preExtendSlides = true;
                })
                .addSpatialMarker(new Vector2d(-60, -9*S), () -> {
                    dropPurple = true;
                })
                .build();
        PreloadDrive3FrontStage = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-47, -14*S, Math.toRadians(176)*S))
                .addSpatialMarker(new Vector2d(-47, -16*S), () -> {
                    preExtendSlides = true;
                })
                .addSpatialMarker(new Vector2d(-45, -15*S), () -> {
                    dropPurple = true;
                })
                .build();

        PreloadDrive3FrontFirst = drive.trajectoryBuilder(startPoseFront)
                .lineToLinearHeading(new Pose2d(-43, -24*S, Math.toRadians(180)*S))
                .build();
        PreloadDrive3FrontSecond = drive.trajectoryBuilder(PreloadDrive3FrontFirst.end())
                .lineToLinearHeading(new Pose2d(-60, -9*S, Math.toRadians(176)*S))
                .build();

        firstDriveThroughStageAfterPurple2 = drive.trajectoryBuilder(PreloadDrive2FrontStage.end(), true)

                .lineToSplineHeading(new Pose2d(20, -9*S, Math.toRadians(180)*S))
                .addSpatialMarker(new Vector2d( 26, -9*S), () -> {
                    extendSlidesAroundTruss = true;
                })
                .splineToConstantHeading(new Vector2d(36, (-9 - 15)*S), Math.toRadians(-42)*S, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        firstDriveThroughStageAfterPurple3 = drive.trajectoryBuilder(PreloadDrive3FrontSecond.end(), true)
                //.lineToSplineHeading(new Pose2d(-5, -9*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(20, -9*S, Math.toRadians(180)*S))
                .addSpatialMarker(new Vector2d( 31, -9*S), () -> {
                    extendSlidesAroundTruss = true;
                })
                .splineToConstantHeading(new Vector2d(36, (-9 - 20)*S), Math.toRadians(-58)*S, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        firstDriveThroughStageAfterPurple1 = drive.trajectoryBuilder(PreloadDrive1FrontStage.end(), true)
                .lineToSplineHeading(new Pose2d(20, -9*S, Math.toRadians(180)*S))
                .addSpatialMarker(new Vector2d( 22, -9*S), () -> {
                    extendSlidesAroundTruss = true;
                })
                .splineToConstantHeading(new Vector2d(36, (-9 - 7)*S), Math.toRadians(-38)*S, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        final double stackXForStage = -28;
        Pose2d frontStageEnd2 = firstDriveThroughStageAfterPurple2.end();
        Pose2d newFrontStageEnd2 = new Pose2d(frontStageEnd2.getX() + 3,frontStageEnd2.getY(),frontStageEnd2.getHeading());
        driveIntoStacksAfterYellowStage2 = drive.trajectoryBuilder(newFrontStageEnd2)
                .splineToConstantHeading(new Vector2d(33, (MiddleLaneYIntake + 10 + 8)*S), Math.toRadians(110)*S, SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstStageDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(18.5, (StageYIntake)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(38, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-17, (StageYIntake)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(stackXForStage, (StageYIntake)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Pose2d frontStageEnd1 = firstDriveThroughStageAfterPurple1.end();
        Pose2d newFrontStageEnd1 = new Pose2d(frontStageEnd1.getX() + 3,frontStageEnd1.getY(),frontStageEnd1.getHeading());
        //TODO change to StageY For other paths as well
        driveIntoStacksAfterYellowStage1 = drive.trajectoryBuilder(newFrontStageEnd1)
                .splineToConstantHeading(new Vector2d(24, (MiddleLaneYIntake + 25)*S), Math.toRadians(170)*S, SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstStageDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(15, (MiddleLaneYIntake + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-17, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(stackXForStage, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Pose2d frontStageEnd3 = firstDriveThroughStageAfterPurple3.end();
        Pose2d newFrontStageEnd3 = new Pose2d(frontStageEnd3.getX() + 3,frontStageEnd3.getY(),frontStageEnd3.getHeading());
        driveIntoStacksAfterYellowStage3 = drive.trajectoryBuilder(newFrontStageEnd3)
                .splineToConstantHeading(new Vector2d(34, (MiddleLaneYIntake + 11)*S), Math.toRadians(110)*S, SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstStageDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(27.8, (MiddleLaneYIntake + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(36, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-17, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(stackXForStage, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // first drive through truss
     ;
        firstDriveThroughTrussAfterPurple2 = drive.trajectoryBuilder(PreloadDrive2Front.end(), true)
                .splineToConstantHeading(new Vector2d(-41, (MiddleLaneYDeposit + 4 - LaneOffsetTruss - FirstTrussDriveOffset)*S), Math.toRadians(-35)*S, SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-18, (MiddleLaneYDeposit - LaneOffsetTruss - FirstTrussDriveOffset)*S), Math.toRadians(0)*S, SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-5, (MiddleLaneYDeposit - LaneOffsetTruss - FirstTrussDriveOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addSpatialMarker(new Vector2d( 23, (MiddleLaneYDeposit - LaneOffsetTruss - FirstTrussDriveOffset)*S), () -> {
                    extendSlidesAroundTruss = true;
                })
                .splineToConstantHeading(new Vector2d(36, (MiddleLaneYDeposit + 20 -LaneOffsetTruss)*S), Math.toRadians(60)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
                //new Pose2d(36,-32.8, Math.toRadians(180)) this is the endpose for this spline
        firstDriveThroughTrussAfterPurple1 = drive.trajectoryBuilder(PreloadDrive1Front.end(), true)
                .splineToConstantHeading(new Vector2d(-41, (MiddleLaneYDeposit + 4 - LaneOffsetTruss - FirstTrussDriveOffset)*S), Math.toRadians(-35)*S, SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-18, (MiddleLaneYDeposit - LaneOffsetTruss- FirstTrussDriveOffset)*S), Math.toRadians(0)*S, SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-5, (MiddleLaneYDeposit - LaneOffsetTruss- FirstTrussDriveOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addSpatialMarker(new Vector2d( 20, (MiddleLaneYDeposit - LaneOffsetTruss- FirstTrussDriveOffset)*S), () -> {
                    extendSlidesAroundTruss = true;
                })
                .splineToConstantHeading(new Vector2d(36, (MiddleLaneYDeposit + 23.5 -LaneOffsetTruss)*S), Math.toRadians(61)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //new Pose2d(36,-32.8 + 3.5, Math.toRadians(180)) this is the endpose for this spline

        firstDriveThroughTrussAfterPurple3 = drive.trajectoryBuilder(PreloadDrive2Front.end(), true)
                .splineToConstantHeading(new Vector2d(-41, (MiddleLaneYDeposit + 4 - LaneOffsetTruss- FirstTrussDriveOffset)*S), Math.toRadians(-35)*S, SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-18, (MiddleLaneYDeposit - LaneOffsetTruss- FirstTrussDriveOffset)*S), Math.toRadians(0)*S, SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-5, (MiddleLaneYDeposit - LaneOffsetTruss- FirstTrussDriveOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addSpatialMarker(new Vector2d( 23, (MiddleLaneYDeposit - LaneOffsetTruss- FirstTrussDriveOffset)*S), () -> {
                    extendSlidesAroundTruss = true;
                })
                .splineToConstantHeading(new Vector2d(36, (MiddleLaneYDeposit + 5 -LaneOffsetTruss)*S), Math.toRadians(40)*S, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //new Pose2d(36,-32.8 - 15, Math.toRadians(180)) this is the endpose for this spline

        //TODO new trajectories driving into truss
        //-52.8
        driveIntoStacksAfterYellowTruss3 = drive.trajectoryBuilder(firstDriveThroughStageAfterPurple3.end())
                .lineToSplineHeading(new Pose2d(30, (-47.8 - 2)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstTrussDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-16.5, (MiddleLaneYIntake - LaneOffsetTruss)*S), Math.toRadians(180)*S) // end tangent affects path alot\
                .splineTo(new Vector2d(-36,-47.8*S), Math.toRadians(StackTrussAngle1)*S, SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // roughly 160
                .build();
        driveIntoStacksAfterYellowTruss2 = drive.trajectoryBuilder(firstDriveThroughStageAfterPurple2.end())
                .lineToSplineHeading(new Pose2d(27, (-47.8 + 5)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstTrussDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-16.5, (MiddleLaneYIntake - LaneOffsetTruss)*S), Math.toRadians(180)*S) // end tangent affects path alot\
                .splineTo(new Vector2d(-36,-47.8*S), Math.toRadians(StackTrussAngle1)*S, SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // roughly 160
                .build();

        driveIntoStacksAfterYellowTruss1 = drive.trajectoryBuilder(firstDriveThroughStageAfterPurple1.end())
                .lineToSplineHeading(new Pose2d(27.5, (-47.8 + 5.3)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstTrussDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-16.5, (MiddleLaneYIntake - LaneOffsetTruss)*S), Math.toRadians(180)*S) // end tangent affects path alot\
                .splineTo(new Vector2d(-36,-47.8*S), Math.toRadians(StackTrussAngle1)*S, SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // roughly 160
                .build();


        double backTrussFirstDriveOffset = 1.8;
        driveIntoStacksAfterBackTruss3 = drive.trajectoryBuilder(firstDriveThroughStageAfterPurple1.end())
                .lineToSplineHeading(new Pose2d(33, (-47.8 + 6.8)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstTrussDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-16.5, (MiddleLaneYIntake - LaneOffsetTruss- backTrussFirstDriveOffset)*S), Math.toRadians(180)*S) // end tangent affects path alot\
                .splineTo(new Vector2d(-36,-47.8*S), Math.toRadians(StackTrussAngle1)*S, SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // roughly 160
                .build();

        driveIntoStacksAfterBackTruss2 = drive.trajectoryBuilder(PreloadDrive2.end())
                .lineToSplineHeading(new Pose2d(27.5, (-47.8 + 5.3)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstTrussDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-16.5, (MiddleLaneYIntake - LaneOffsetTruss- backTrussFirstDriveOffset)*S), Math.toRadians(180)*S) // end tangent affects path alot\
                .splineTo(new Vector2d(-36,-47.8*S), Math.toRadians(StackTrussAngle1)*S, SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveIntoStacksAfterBackTruss1 = drive.trajectoryBuilder(PreloadDrive1.end())
                .lineToSplineHeading(new Pose2d(27.5, (-47.8 + 5.3)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstTrussDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-16.5, (MiddleLaneYIntake - LaneOffsetTruss - backTrussFirstDriveOffset)*S), Math.toRadians(180)*S) // end tangent affects path alot\
                .splineTo(new Vector2d(-36,-47.8*S), Math.toRadians(StackTrussAngle1)*S, SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveIntoStacksAfterBackStage1 = drive.trajectoryBuilder(PreloadDrive1.end())
                .lineToSplineHeading(new Pose2d(33, (MiddleLaneYIntake + LaneOffset - 10)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstStageDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(33, (-32 + 11)), Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(17, (MiddleLaneYIntake + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-17, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-28.7, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //-52.8
        driveIntoStacksAfterBackStage2 = drive.trajectoryBuilder(PreloadDrive2.end())
                .lineToSplineHeading(new Pose2d(33, (MiddleLaneYIntake + LaneOffset - 10)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstStageDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(33, (-32 + 11)), Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(12, (MiddleLaneYIntake + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(38, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-17, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-28.7, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        driveIntoStacksAfterBackStage3 = drive.trajectoryBuilder(PreloadDrive3.end())
                .lineToSplineHeading(new Pose2d(33, (MiddleLaneYIntake + LaneOffset - 10)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityFirstStageDriveToStacks, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(33, (-32 + 11)), Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(5, (MiddleLaneYIntake + LaneOffset)*S), Math.toRadians(180)*S, SampleMecanumDrive.getVelocityConstraint(38, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-17, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-28.7, (MiddleLaneYIntake + LaneOffset)*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(SlowerVelocityConstraintIntake, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


      /*  AfterPreloadDrive1Front = drive.trajectoryBuilder(PreloadDrive1Front.end())
                .lineToLinearHeading(new Pose2d(-54, FrontPreloadY*S, Math.toRadians(180)*S)) // hardest one closest to stack
                .build();
        AfterPreloadDrive2Front = drive.trajectoryBuilder(PreloadDrive2Front.end())
                .lineToLinearHeading(new Pose2d(-45, FrontPreloadY*S, Math.toRadians(180)*S)) // hardest one closest to stack
                .build();
        AfterPreloadDrive3Front = drive.trajectoryBuilder(PreloadDrive3Front.end())
                .lineToLinearHeading(new Pose2d(-38, FrontPreloadY*S, Math.toRadians(180)*S)) // hardest one closest to stack
                .build();*/
    }

    /* //shouldn't need to do this if drive is public :)
    public Pose2d getPoseEstimate(){
        return drive.getPoseEstimate();
    }
     */

    // might have to make multiple of these maybe
    public void driveIntoStackStraight(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage, double positiveYOffset){ // generates a live trajectory
        Trajectory driveIntoStackStraightLIVE = drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(9, (MiddleLaneYIntake +positiveYOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffsetTruss)))*S, Math.toRadians(180)*S))
                .splineToConstantHeading(new Vector2d(-25,(MiddleLaneYIntake +positiveYOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffsetTruss)))*S ), Math.toRadians(180)*S) // end tangent affects path alot\
                .lineToSplineHeading(new Pose2d(-28, (MiddleLaneYIntake +positiveYOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffsetTruss)))*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
        drive.followTrajectoryAsync(driveIntoStackStraightLIVE);
    }
    //Stage
    public Trajectory driveIntoStackStraightTrajectory(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage, double positiveYOffset, double intakeX, double slowedX, double heading){ // generates a live trajectory
        return drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(10, (StageYIntake +positiveYOffset)*S, Math.toRadians(heading)*S), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(slowedX, (StageYIntake +positiveYOffset)*S), Math.toRadians(heading)*S) // end tangent affects path alot\
                .lineToSplineHeading(new Pose2d(intakeX, (StageYIntake +positiveYOffset)*S, Math.toRadians(heading)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // seperates trajectories
                .build();
    }

    public void goBackToStack(Pose2d startTrajectory, double slowerVelocityIntoStack, int trussMiddleStage, double xPosition){ // generates a live trajectory
        Trajectory driveBackIntoStack = drive.trajectoryBuilder(startTrajectory)
                .lineToLinearHeading(new Pose2d(xPosition,(MiddleLaneYIntake + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffsetTruss)))*S, Math.toRadians(180)*S), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
    public Trajectory outtakeDriveFromStraightTUrnEndStageV2Trajectory(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double yOffset, double xSplineValue){
        return drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(-8, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(180)*S))
                .splineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(endAngle)*S),Math.toRadians(0)*S)
                .lineToSplineHeading(new Pose2d(xSplineValue, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(endAngle)*S))
                .splineToConstantHeading(new Vector2d(28, (MiddleLaneYDeposit - yOffset + LaneOffset)*S),Math.toRadians(-20)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }
    public Trajectory simplifiedOuttakeDrive(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double endTangent, double yOffset, double xSplineValue, double xEnd){
        return drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(xSplineValue, (MiddleLaneYDeposit + LaneOffset)*S, Math.toRadians(endAngle)*S))
                .splineToConstantHeading(new Vector2d(xEnd, (MiddleLaneYDeposit - yOffset+ LaneOffset)*S), Math.toRadians(endTangent), SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // end tangent of path
                .build();
    }

    public Trajectory driveIntoStackAngledAfterAngledOuttakeTrajectory(Pose2d startTrajectory, double slowerVelocityIntoStack, double yOffset, double endAngle, int trussMiddleStage, double positiveYDriftOffset, double intakeX){
        trussSideStackCoordinate = new Pose2d(intakeX,(MiddleLaneYDeposit+positiveYDriftOffset+ -LaneOffsetTruss + yOffset),endAngle);

        return drive.trajectoryBuilder(startTrajectory)
                // FOR THAT ONE CASE MOVE BACK SO WE DONT HIT THE TRUSS                                                                         // the below code adds an offset so it doesn't swing as far for the first cycle specifically
                .lineToSplineHeading(new Pose2d((teamPropLocation == 3 && trussMiddleStage == 1)||(teamPropLocation == 3 && trussMiddleStage == 3)? 27:18, (MiddleLaneYDeposit+positiveYDriftOffset + (numCycles == 0 && trussMiddleStage == 1? 5:-1) + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffsetTruss)))*S, Math.toRadians(180)*S),SampleMecanumDrive.getVelocityConstraint(trussMiddleStage == 1 || (trussMiddleStage == 3 && teamPropLocation == 3)? 35: DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-16.5, (MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffsetTruss)))*S), Math.toRadians(180)*S) // end tangent affects path alot\

                /*.addSpatialMarker(new Vector2d(-24.5, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), () -> {
                    extendSlidesAroundTruss = true;
                })*/
               /* .addSpatialMarker(new Vector2d(-11, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), () -> {
                    extendSlidesAroundStage = true;
                })

                */
                .splineTo(new Vector2d(intakeX,(MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss: LaneOffsetTruss)) + yOffset)*S), Math.toRadians(endAngle)*S,SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // roughly 160
                .build();

    }


    public Trajectory driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(Pose2d startTrajectory, double slowerVelocityIntoStack, double yOffset, double endAngle, int trussMiddleStage, double positiveYDriftOffset, double xSplineStart){
        return drive.trajectoryBuilder(startTrajectory)
                .lineToSplineHeading(new Pose2d(15, (StageYIntake+positiveYDriftOffset)*S, Math.toRadians(180)*S),SampleMecanumDrive.getVelocityConstraint( 30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(xSplineStart, (StageYIntake+positiveYDriftOffset)*S), Math.toRadians(180)*S) // end tangent affects path alot\
                /*
                 .addSpatialMarker(new Vector2d(-20, MiddleLaneYDeposit+positiveYDriftOffset+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S))), () -> {
                     extendSlidesAroundStage = true;
                 })

                 */
                .splineTo(new Vector2d(-34.7,(StageYIntake+positiveYDriftOffset + yOffset)*S), Math.toRadians(endAngle)*S,SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoStack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) // roughly 160
                .build();
    }
    public Trajectory driveBackToDropYellow(Pose2d startTrajectory, double slowerVelocity, double xOffset){
        return drive.trajectoryBuilder(startTrajectory)

                .lineTo(new Vector2d(startTrajectory.getX() + xOffset,startTrajectory.getY()),SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }


    public Trajectory outtakeDriveFromAngleTurnEndTrajectory(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double depositX, double endAngleOrTangent, double yOffset, int trussMiddleStage, double outtakeDriveOffset, double xSplinePosition){
        return drive.trajectoryBuilder(startTrajectory, true)

                .splineTo(new Vector2d(-18,(MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss - outtakeDriveOffset: LaneOffsetTruss- outtakeDriveOffset)))*S), Math.toRadians(0)*S,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addSpatialMarker(new Vector2d( 5, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss - outtakeDriveOffset: LaneOffsetTruss - outtakeDriveOffset)))*S), () -> {
                    extendSlidesAroundTruss = true;
                })
                .lineToSplineHeading(new Pose2d(xSplinePosition, (MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss - outtakeDriveOffset: LaneOffsetTruss- outtakeDriveOffset)))*S, Math.toRadians(180)*S))
                //.splineTo(new Vector2d(depositX,(MiddleLaneYDeposit+ (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss*S: LaneOffset*S)) + yOffset)*S), Math.toRadians(endAngle)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(depositX,(MiddleLaneYDeposit + (trussMiddleStage == 2? 0: (trussMiddleStage == 1? -LaneOffsetTruss - outtakeDriveOffset: LaneOffsetTruss- outtakeDriveOffset))+ yOffset)*S),Math.toRadians(endAngleOrTangent)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
    }

    // v1 - for other purple pixel cases


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
                .lineToLinearHeading(new Pose2d(ParkX, wallOrMiddle == 1?ParkWallY*S:ParkMiddleY*S,Math.toRadians(180)*S)) // hardest one closest to stack
                .build();
    }
    /*
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

                *//*
                .splineToConstantHeading(new Vector2d(15, -36*S),Math.toRadians(180)*S)
                .lineToSplineHeading(new Pose2d(10, -36*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-15, -36*S, Math.toRadians(180)*S))
                .lineToSplineHeading(new Pose2d(-18, -36*S, Math.toRadians(160)*S))
                .splineToConstantHeading(new Vector2d(-36, -32*S),Math.toRadians(160)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 *//*

                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
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
    public void outtakeDriveFromAngleTurnEnd(Pose2d startTrajectory, double slowerVelocityIntoBackdrop, double endAngle, double yOffset){
        Trajectory outtakeDriveTurnEndV2 = drive.trajectoryBuilder(startTrajectory, true)

                .splineTo(new Vector2d(-18,(MiddleLaneYDeposit+ LaneOffset)*S), Math.toRadians(0)*S)
                .lineToSplineHeading(new Pose2d(10, (MiddleLaneYDeposit+ LaneOffset)*S, Math.toRadians(0)*S))
                .splineTo(new Vector2d(32,(MiddleLaneYDeposit+ LaneOffset + yOffset)*S), Math.toRadians(endAngle)*S, SampleMecanumDrive.getVelocityConstraint(slowerVelocityIntoBackdrop, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        drive.followTrajectoryAsync(outtakeDriveTurnEndV2);
        */
}
