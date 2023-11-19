package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.system.accessory.profile.FeedForward;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Disabled
public class ProfileTuning extends LinearOpMode
{
    // Values in Ticks
    public static double distance = 800;
    public static double vel = 1, accel = 1, decel = 1;
    public static double kP = 0, kI = 0, kD = 0, integralSum = 0, kF = 0;
    public static double fMin = 0, fMax = 0;
    public static boolean reverse;
    private double target, position;
    private FeedForward feedForward;
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    private boolean renew;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Telemetry telemetry1 = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        NanoClock clock = NanoClock.system();

        outtakeSubsystem.initOuttake(hardwareMap);
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();

        update();

        telemetry.addLine("Press play to begin the profile tuning routine");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested())
        {
            outtakeSubsystem.outtakeReads();
            outtakeSubsystem.pitchToInternalPID(300, 0.65); // so the sliders don't fall
            update();
            if (renew)
            {
                if (reverse)
                {
                    target = 0;
                    feedForward.resetTime();
                } else
                {
                    target = distance;
                    feedForward.resetTime();
                }
                renew = false;
            }
            feedForward.setProfile(profile);
            feedForward.setTarget(target);
            feedForward.setPos(outtakeSubsystem.liftPosition);
            feedForward.reads(outtakeSubsystem.liftPosition, profile);
            double output = feedForward.calculateFullState();


        }
    }
    private void update()
    {

        this.constraints = new ProfileConstraints(vel, accel, decel);
        this.profile = new AsymmetricMotionProfile(position, target, constraints);
        this.feedForward = new FeedForward(FeedForward.FeedForwardMode.CONSTANT, fMin, fMax);
        feedForward.setPID(kP, kI, kD, integralSum, kF);
    }
}
