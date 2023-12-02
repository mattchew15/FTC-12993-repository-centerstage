/*
package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.system.accessory.ToggleR;
import org.firstinspires.ftc.teamcode.system.accessory.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileSubsystem;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Disabled
@Config
@TeleOp(name = "SimplicityDriveMotion", group = "TestR")
public class SimplicityDriveMotion extends LinearOpMode {


    // Full state feedback
    public static double kPos = 0.2, kVel = 0.2;
    // Feed forward
    public static double kP = 0.015, kI = 0.0001, kD = 0.00006, kF = 0.01;
    //LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00006, LiftIntegralSumLimit = 10, LiftKf = 0;
    //Subsystems
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    AsymmetricMotionProfile profileSlider;
    public static ProfileConstraints profileSliderConstraints = new ProfileConstraints(11000, 11000, 11000);
    // Max ticks of the motor is like 2800, 1150rpm, 19.16 rps, 145 ticks/rev, = 2779 is the max vel ticks/second, accel should be vel / time(s)
    ElapsedTime GlobalTimer = new ElapsedTime();
    ToggleR toggle = new ToggleR();
    // Don't use the constructor with the telemetry in final
    public static ProfileSubsystem profileSubsystem = new ProfileSubsystem(ProfileSubsystem.SubsystemMode.NULL, 0.04, 0.06);
    double target;

    @Override
    public void runOpMode() throws InterruptedException
    {
        outtakeSubsystem.initOuttake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        GlobalTimer.reset();
        GlobalTimer.startTime();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();
        profileSubsystem.setPID(kP, kI, kD, 10);


        waitForStart();

        // Yeah i know it is crappy it is just for testing the profiles
        while (opModeIsActive() && !isStopRequested())
        {
            outtakeSubsystem.outtakeReads(false);
            outtakeSubsystem.pitchToInternalPID(300, 0.65); // so the sliders don't fall
            // Implementation needs a toggle so it doesn't keep resetting the time

            if (toggle.mode(gamepad1.a))
            {
                target = 200;
                profileSubsystem.resetTime();
            }
            if (toggle.mode(gamepad1.b))
            {
                target = 400;
                profileSubsystem.resetTime();

            }
            if (toggle.mode(gamepad1.y))
            {
                target = 800;
                profileSubsystem.resetTime();

            }
            if (toggle.mode(gamepad1.x))
            {
                target = 0;
                profileSubsystem.resetTime();

            }


            // In final implementation creating a profile to every height should be better, pls don't do this spaghetti code
            // Actually I am not sure anymore, this might be better. figure out a way to stopped
            // the motor at zero. Nvm just use the has reached when the target is 0, this make the previous line useless
            // I should stop writing this comments

            //profileSlider = new AsymmetricMotionProfile(outtakeSubsystem.liftPosition, target, profileSliderConstraints);
            // I made a function that should condense this, not tested tho
            //profileSubsystem.setProfile(profileSlider);
            profileSubsystem.setTarget(target);
            profileSubsystem.setPos(outtakeSubsystem.liftPosition);
            profileSubsystem.reads(outtakeSubsystem.liftPosition, profileSlider);
            double output = profileSubsystem.calculateFullState();
            //double output = profileSubsystem.calculateFeedForward();

            double x = profileSlider.state.x;
            double v = profileSlider.state.v;
            double a = profileSlider.state.a;
            // maybe i can place the feed forward inside the outtake, don't think is a good idea tho
            outtakeSubsystem.rawLift(output);

            driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            // Debug telemetry,
            telemetry.addData("Toggle", toggle.mode(gamepad1.right_bumper));
            telemetry.addData("X", x);
            telemetry.addData("V", v);
            telemetry.addData("A", a);
            telemetry.addData("Output", output);
            telemetry.addData("Target", target);
            telemetry.addData("SliderPos", outtakeSubsystem.liftPosition);
            telemetry.update();
        }
    }

}
*/

package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.system.accessory.ToggleR;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;


@Config
@TeleOp(name = "SimplicityDriveMotion", group = "TestR")
public class SimplicityDriveMotion extends LinearOpMode {


    // Feed forward
    public static double kP = 0.015, kI = 0.0001, kD = 0.00006;
    //LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00006, LiftIntegralSumLimit = 10, LiftKf = 0;
    //Subsystems
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    // Max ticks of the motor is like 2800, 1150rpm, 19.16 rps, 145 ticks/rev, = 2779 is the max vel ticks/second, accel should be vel / time(s)
    ElapsedTime GlobalTimer = new ElapsedTime();
    ToggleR toggle = new ToggleR();
    // Don't use the constructor with the telemetry in final
    double target;
    private int cyclesInSync;

    @Override
    public void runOpMode() throws InterruptedException
    {
        outtakeSubsystem.initOuttake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        GlobalTimer.reset();
        GlobalTimer.startTime();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();
        outtakeSubsystem.setLiftFullStateGains(0.0001, 0.0004);
        outtakeSubsystem.profileLiftSetUp();
        waitForStart();

        // Yeah i know it is crappy it is just for testing the profiles
        while (opModeIsActive() && !isStopRequested())
        {
            outtakeSubsystem.outtakeReads(false);
            outtakeSubsystem.pitchToInternalPID(300, 0.65); // so the sliders don't fall
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
            // Implementation needs a toggle so it doesn't keep resetting the time

            if (gamepad1.a)
            {
               outtakeSubsystem.updateLiftTargetProfile(200);
            }
            if (gamepad1.b)
            {
               outtakeSubsystem.updateLiftTargetProfile(400);

            }
            if (gamepad1.y)
            {
               outtakeSubsystem.updateLiftTargetProfile(600);

            }
            if (gamepad1.x)
            {
                outtakeSubsystem.updateLiftTargetProfile(0);

            }
            if (gamepad1.dpad_left)
            {
                outtakeSubsystem.rawLift(0);
            }
            double output = outtakeSubsystem.profileLiftCalculateFeedForward();

            driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            // Debug telemetry,
            //telemetry.addData("Toggle", toggle.mode(gamepad1.right_bumper));
            telemetry.addData("X", outtakeSubsystem.profileSubsystem.getX()); // Think as this as a function
            // telemetry.addData("V", v);
            // telemetry.addData("A", a);
            telemetry.addData("Output Feedforward", output);
            telemetry.addData("Output FullState", outtakeSubsystem.profileLiftCalculate());
            telemetry.addData("Target", outtakeSubsystem.liftTarget);
            telemetry.addData("SliderPos", outtakeSubsystem.liftPosition);
            telemetry.addData("LiftProfile Pos", outtakeSubsystem.liftProfileStartingPosition);
            telemetry.addData("Has reached", outtakeSubsystem.profileSubsystem.hasReached());
            telemetry.addData("Timer profile", outtakeSubsystem.profileSubsystem.getTime());
            boolean sync = false;
            if (outtakeSubsystem.profileSubsystem.hasReached() && Math.abs(outtakeSubsystem.liftTarget - outtakeSubsystem.liftPosition) < 20)
            {
                sync = true;
                cyclesInSync += 1;
            }
            telemetry.addData("Sync", sync);
            telemetry.addData("Cycles in sync", cyclesInSync);
            telemetry.addData("Error", Math.abs(outtakeSubsystem.liftPosition - outtakeSubsystem.profileSubsystem.getX()));
            telemetry.update();
        }
    }

}





