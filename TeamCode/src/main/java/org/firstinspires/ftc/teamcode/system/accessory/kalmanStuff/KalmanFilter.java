/*_
package org.firstinspires.ftc.teamcode.system.accessory.kalmanStuff;

// Attempt to write a kalman filter, control award bait??

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

//Finally, that leaves the model and sensor covariances known as Q and R. These are values that we,
// the implementor of the Kalman filter, have to determine. Large values of Q correspond with high
// trust in the model (estimate before feedback) accuracy, and large values of R correspond with
// high accuracy of the sensor value Z. The process of using the DARE (discrete-time algebraic
// Riccati equation) to find P is done to optimize for a value of K that minimizes a cost function
// constrained by Q, and R. Changing Q and R will change how the Kalman gain is computed and will
// therefore directly impact your sensor performance.
public class KalmanFilter
{
    private final Object sync = new Object();
    private volatile double x = 0; // initial state
    private final double Q = 0.1; // model covariance
    private final double  R = 0.4; // sensor covariance
    private double p = 1; // initial covariance guess
    private double k = 1; // initial Kalman gain guess

    private double x_previous = x;
    private double p_previous = p;
    // leave as global, global variables never hurt right?
    private double u; // Change in state
    private double z;
    private Pose2d actualPos, predictedPos, measuredPos;
    // This regard a single input single output system so it is simplified
    public double filter(double firstSensor, double secondSensor)
    {
        synchronized (sync)
        {
                    // Get the reading of the first sensor (ex. change of pos in odometry)
            u = firstSensor; // This should be the previous output on the system
            x = x_previous + u; // Predicted current state, given by previous state + the previous output

            p = p_previous + Q; // Project the covariance, given by the previous +  system covariance gain
            k = p / (p + R); // Calculate the kalman gain, given by model covariance divided by the model covariance + the sensor covariance

            // Get the reading of the second sensor (ex. april tag)
            z = secondSensor;

            x = x + k * (z - x); // Perform feedback on the sensor reference, given by predicted current state + kalman gain * (the error between second sensor and current state)

            p = (1 - k) * p; // For last we update the model covariance

            p_previous = p;
            x_previous = x;
        }
        return x;
    }

    // Idea is to independently pass the components of the vector individually
    private Pose2d getActualPos()
    {
        return this.actualPos;
    }
    private Pose2d getPredictedPos()
    {
        return this.predictedPos;
    }
    private Pose2d getMeasuredPos()
    {
        return this.measuredPos;
    }
    private double getX(Pose2d pose2d)
    {
        double x;
        synchronized (sync)
        {
            x = pose2d.getX();
        }
        return x;
    }
    private double getY(Pose2d pose2d)
    {
        double y;
        synchronized (sync)
        {
            y = pose2d.getY();
        }
        return y;
    }
    private double getTheta(Pose2d pose2d)
    {
        double theta;
        synchronized (sync)
        {
            theta = pose2d.getHeading();
        }
        return theta;
    }

}
*/

