package org.firstinspires.ftc.teamcode.system.paths.P2P;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;

import java.util.Arrays;
import java.util.List;

public class TwoTrackingWheelLocalizer extends com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
{
    private ImuThread imuThread;
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -0.2; // X is the up and down direction
    public static double PARALLEL_Y =  -6.25; // Y is the strafe direction

    public static double PERPENDICULAR_X = 5.5;
    public static double PERPENDICULAR_Y = -6.25;

    public static double X_MULTIPLIER = 1.007462686567; // Multiplier in the X direction
    // 0.9919177076, 0.9920999449, 0.9917355372, 1.0005558643691, 0.9992598075
    // 72 - 18 = 54
    // 72 - 15 = 57
    public static double Y_MULTIPLIER = 0.974495158; // Multiplier in the Y direction
    // 1.000702247191, 0.9933774834, 0.974025974, 0.9734991888, 0.975433526, 0.974025974

    //16.5 x 14.5

    //Per: 13x 10.5 = x = 5.5 y = -3
    //Par: x = -0.2 y = -6.25

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;


    public TwoTrackingWheelLocalizer(HardwareMap hardwareMap, ImuThread imuThread) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.imuThread = imuThread;
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));

        // reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return imuThread.getImuAngle();
    }

    @Override
    public Double getHeadingVelocity() {
        return imuThread.getImuExternalVel();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        //  If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }
    //this is just gonna use rr but with the thread imu
}
