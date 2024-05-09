package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.system.accessory.kalmanStuff.Kalman;

import java.util.List;
@Deprecated
public class KFLocalizer extends StandardTrackingWheelLocalizer
{
    Kalman kalman = new Kalman(new SimpleMatrix(new double[][]{
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0}
    }));
    public KFLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels)
    {
        super(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
    }
    @Override
    public void update()
    {
        super.update();
    }
    public void updateKF()
    {

    }

}