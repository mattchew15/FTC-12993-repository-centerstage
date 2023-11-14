package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.system.accessory.FullStateFeedback;
import org.firstinspires.ftc.teamcode.system.accessory.Toggle;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleR;
import org.firstinspires.ftc.teamcode.system.accessory.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
@TeleOp(name = "SimplicityDriveMotion")
public class SimplicityDriveMotion extends LinearOpMode {

    enum ProfileStateFSM{
        START,
        TRAJECTORY,
        END
    }
    // Full state feedback
    public static double kPos = 0.01, kVel = 0.01;
    // Feed forward
    public static double kP = 0.5, kV = 0.2, kA = 0;
    //Subsystems
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    AsymmetricMotionProfile profileSlider;
    public static ProfileConstraints profileSliderConstraints = new ProfileConstraints(1800, 4000, 2000);
    FullStateFeedback fullStateFeedback = new FullStateFeedback(kPos, kVel);
    ElapsedTime GlobalTimer = new ElapsedTime();
    ElapsedTime stateTimer = new ElapsedTime();
    ToggleR toggle = new ToggleR();


    double target;
    @Override
    public void runOpMode() throws InterruptedException
    {
        outtakeSubsystem.initOuttake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        GlobalTimer.reset();
        GlobalTimer.startTime();
        stateTimer.reset();
        stateTimer.startTime();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();
        ProfileStateFSM state = ProfileStateFSM.START;
        waitForStart();

        // Yeah i know it is crappy it is just for testing the profiles
        while (opModeIsActive() && !isStopRequested())
        {
            // This shitty implementation needs a toggle so it doesn't keep resseting the time
            if (toggle.mode(gamepad1.a))
            {
                target = 200;
                stateTimer.reset();
            }
            if (toggle.mode(gamepad1.b))
            {
                target = 400;
                stateTimer.reset();
            }
            if (toggle.mode(gamepad1.y))
            {
                target = 600;
                stateTimer.reset();
            }
            if (toggle.mode(gamepad1.x))
            {
                target = 0;
                stateTimer.reset();
            }
            /*
            switch (state)
            {
                case START:
                    if (target != 0)
                    {
                        state = ProfileStateFSM.TRAJECTORY;
                        profileSlider = new AsymmetricMotionProfile(outtakeSubsystem.liftPosition, target, profileSliderConstraints);
                    }
                    stateTimer.reset();
                    stateTimer.startTime();
                    break;
                case TRAJECTORY:
                    if (target == 0)
                    {
                        state = ProfileStateFSM.END;
                    }
                    profileSlider.calculate(stateTimer.seconds());
                    break;
                case END:
                    profileSlider = new AsymmetricMotionProfile(outtakeSubsystem.liftPosition, target, profileSliderConstraints);

                    break;

            }

             */
            // In final implementation creating a profile to every height should be better, pls don't do this spaghetti code
            profileSlider = new AsymmetricMotionProfile(outtakeSubsystem.liftPosition, target, profileSliderConstraints);
            profileSlider.calculate(stateTimer.seconds());

            double x = profileSlider.state.x;
            double v = profileSlider.state.v;
            double a = profileSlider.state.a;


            // Feedforward implementation
            outtakeSubsystem.rawLift(x * kP + v * kV + a * kA);
            // TODO: check the implementation of the fullStateFeedback...
            //outtakeSubsystem.rawLift(fullStateFeedback.update(target, x, v)); Test the profile before this...
            driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("Target", target);
            telemetry.addData("SliderPos", outtakeSubsystem.liftPosition);
            telemetry.update();
        }
    }
}



