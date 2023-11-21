package org.firstinspires.ftc.teamcode.system.accessory.kalmanStuff.Michael;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.simple.SimpleMatrix;

/**
 * Extended Kalman Filter (EKF) for sensor fusion between odometry and distance sensor
 * Odometry is used as prediction to generate estimate
 * Distance Sensor is used as measurement to generate correction
 */
@Config
public class Kalman2 {

    public SimpleMatrix[] pose; // pose, cov

    // values
    private final SimpleMatrix STARTING_COVARIANCE = new SimpleMatrix(new double[][]{
            {0.2, 0, 0},
            {0, 0.05, 0},
            {0, 0, Math.toRadians(0.5)}
    });

    private SimpleMatrix P;

    public Kalman2(SimpleMatrix[] pose) {
        this.pose = pose;
    }

    public Kalman2(SimpleMatrix pose, SimpleMatrix cov) {
        this.pose = new SimpleMatrix[]{pose,cov};
    }

    public Kalman2(SimpleMatrix pose) {
        this.pose = new SimpleMatrix[]{pose,STARTING_COVARIANCE};
    }

    public void update(SimpleMatrix update, SimpleMatrix obs){
        if (update != null && obs == null)
        {
            pose = prediction(pose,update);
        }
        else if (update == null && obs != null)
        {
            pose = correction(pose, obs);
        }
        else // if (update != null && obs != null)
        {
            pose = fuse(pose, update, obs);
        }
    }

    public SimpleMatrix[] prediction(SimpleMatrix[] prev, SimpleMatrix update) {
        // TODO: are this values used?
        // set up
        double prevX = prev[0].get(0,0);
        double prevY = prev[0].get(1,0);
        double prevHeading = prev[0].get(2,0);
        SimpleMatrix prevCov = prev[1];

        SimpleMatrix F = new SimpleMatrix(new double[][]{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        });
        //TODO: tweak Windows x64 (tested)
        //Windows x32 (untested)
        //MacOS x64 (tested)
        //Linux x64 (tested for Ubuntu 20.04)

        SimpleMatrix Q = new SimpleMatrix(new double[][]{
                {0.3, 0, 0},
                {0, 0.2, 0},
                {0, 0, 0.9}
        });
        P = F.mult(prevCov).mult(F.transpose()).plus(Q);

        // calculate
        double predX = update.get(0, 0);
        double predY = update.get(1,0);
        double predHeading = angleWrap(update.get(2,0));

        return new SimpleMatrix[]{
                new SimpleMatrix(new double[][]{
                        {predX},
                        {predY},
                        {predHeading}
                }),
                P
        };
    }
    //obs = observation
    public SimpleMatrix[] correction(SimpleMatrix[] pred, SimpleMatrix obs) {
        // set up
        double tX = obs.get(0,0);
        double tY = obs.get(1,0);
        double tPhi = obs.get(2,0);
        SimpleMatrix predCov = pred[1];

        SimpleMatrix i;
        SimpleMatrix z = new SimpleMatrix(new double [][]{
                {tX},
                {tY},
                {tPhi}
        });
        SimpleMatrix H = new SimpleMatrix(new double[][]{
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        });

        SimpleMatrix w = new SimpleMatrix(new double[][]{
                {0.4},
                {0.4},
                {0.0001}
        });
        i = z.minus(H.mult(pred[0]).plus(w));
        //sensor noise matrix
        SimpleMatrix Q = new SimpleMatrix(new double[][]{
                {0.2588293736, 0, 0},
                {0, 0.269458943, 0},
                {0, 0, 0.01}
        }); // random

        // TODO: simplify this??
        SimpleMatrix S = H.mult(predCov).mult(H.transpose()).plus(Q);
        SimpleMatrix K = predCov.mult(H.transpose().mult(S.invert()));

        SimpleMatrix correct = pred[0].plus(K.mult(i));
        correct.set(2,0, angleWrap(correct.get(2,0)));
        SimpleMatrix correctCov = predCov.minus(K.mult(H).mult(predCov));

        return new SimpleMatrix[]{correct,correctCov};
    }

    public SimpleMatrix[] fuse(SimpleMatrix[] prev, SimpleMatrix update, SimpleMatrix obs) {
        SimpleMatrix[] prediction = prediction(prev, update);
        SimpleMatrix[] correction = correction(prediction, obs);
        return correction;
    }

    private double angleWrap(double input) {
        input %= 360;
        if (input > 180) input -= 360;
        return input;
    }

    public Pose2d getPose() {
        double x = pose[0].get(0, 0);
        double y = pose[0].get(1, 0);
        double z = pose[0].get(2, 0);
        return new Pose2d(x, y, z);
    }
}
