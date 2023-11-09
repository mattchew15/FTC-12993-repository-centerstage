package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.system.accessory.FullStateFeedback;
import org.firstinspires.ftc.teamcode.system.accessory.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
@TeleOp(name = "SimplicityDrive")
public class SimplicityDriveMotion extends LinearOpMode {

    enum ProfileStateFSM{
        START,
        TRAJECTORY,
        END
    }
    // Full state feedback
    public static double kPos = 0.01, kVel = 0.01;
    // Feed forward
    public static double kP = 0.1, kV = 0.01, kA = 0;
    //Subsystems
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    AsymmetricMotionProfile profileSlider;
    ProfileConstraints profileSliderConstraints = new ProfileConstraints(5, 2,2);
    FullStateFeedback fullStateFeedback = new FullStateFeedback(kPos, kVel);
    ElapsedTime GlobalTimer, stateTimer;


    double target;
    @Override
    public void runOpMode() throws InterruptedException
    {
        GlobalTimer.reset();
        GlobalTimer.startTime();
        stateTimer.startTime();
        waitForStart();
        outtakeSubsystem.encodersReset();
        ProfileStateFSM state = ProfileStateFSM.START;

        // Yeah i know it is crappy it is just for testing the profiles
        if(opModeIsActive())
        {
            if (gamepad1.a)
            {
                target = 200;
                stateTimer.reset();
            }
            if (gamepad1.b)
            {
                target = 400;
                stateTimer.reset();
            }
            if (gamepad1.y)
            {
                target = 600;
                stateTimer.reset();
            }
            if (gamepad1.x)
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

        }
    }
}



