package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.system.accessory.FullStateFeedback;
import org.firstinspires.ftc.teamcode.system.accessory.Toggle;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleR;
import org.firstinspires.ftc.teamcode.system.accessory.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.system.accessory.profile.FeedForward;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
@TeleOp(name = "SimplicityDriveMotion")
public class SimplicityDriveMotion extends LinearOpMode {


    // Full state feedback
    public static double kPos = 0.2, kVel = 0.2;
    // Feed forward
    public static double kP = 0.015, kI = 0.0, kD = 0.00006;
    //LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00006, LiftIntegralSumLimit = 10, LiftKf = 0;
    //Subsystems
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    AsymmetricMotionProfile profileSlider;
    public static ProfileConstraints profileSliderConstraints = new ProfileConstraints(2000, 1950, 950);
    // Max ticks of the motor is like 2800, 1150rpm, 19.16 rps, 145 ticks/rev, = 2778
    FullStateFeedback fullStateFeedback = new FullStateFeedback(kPos, kVel); // this should be delt inside the profile calculation
    ElapsedTime GlobalTimer = new ElapsedTime();
    ToggleR toggle = new ToggleR();
    // Don't use the constructor with the telemetry in final
    FeedForward feedForward = new FeedForward(FeedForward.FeedForwardMode.CONSTANT, 0.03, 0.05);
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
        feedForward.setPID(kP, kI, kD);


        waitForStart();

        // Yeah i know it is crappy it is just for testing the profiles
        while (opModeIsActive() && !isStopRequested())
        {
            outtakeSubsystem.outtakeReads();
            outtakeSubsystem.pitchToInternalPID(300, 0.65); // so the sliders don't fall
            // Implementation needs a toggle so it doesn't keep resetting the time

            if (toggle.mode(gamepad1.a))
            {
                target = 200;
                feedForward.resetTime();
                feedForward.updateTarget(200);

            }
            if (toggle.mode(gamepad1.b))
            {
                target = 400;
                feedForward.resetTime();

            }
            if (toggle.mode(gamepad1.y))
            {
                target = 800;
                feedForward.resetTime();

            }
            if (toggle.mode(gamepad1.x))
            {
                target = 0;
                feedForward.resetTime();

            }


            // In final implementation creating a profile to every height should be better, pls don't do this spaghetti code
            // Actually I am not sure anymore, this might be better. figure out a way to stopped
            // the motor at zero. Nvm just use the has reached when the target is 0, this make the previous line useless
            // I should stop writing this comments

            profileSlider = new AsymmetricMotionProfile(outtakeSubsystem.liftPosition, target, profileSliderConstraints);
            // I made a function that should condense this, not tested tho
            feedForward.setProfile(profileSlider);
            feedForward.setTarget(target);
            feedForward.setPos(outtakeSubsystem.liftPosition);
            feedForward.reads(outtakeSubsystem.liftPosition, profileSlider);
            double output = feedForward.calculateFeedForward();

            double x = profileSlider.state.x;
            double v = profileSlider.state.v;
            double a = profileSlider.state.a;
            // maybe i can place the feed forward inside the outtake, don't think is a good idea tho
            outtakeSubsystem.rawLift(output);
/*
            double output = x * kP + v * kV + a * kA; //fullStateFeedback.update(target, x, v);
            // Feedforward implementation
            //outtakeSubsystem.rawLift(output);
            // TODO: check the implementation of the fullStateFeedback...
            outtakeSubsystem.rawLift(output); // Test the profile before this...

 */
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



