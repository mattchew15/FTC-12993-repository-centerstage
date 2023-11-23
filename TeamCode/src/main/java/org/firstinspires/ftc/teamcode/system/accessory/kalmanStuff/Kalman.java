package org.firstinspires.ftc.teamcode.system.accessory.kalmanStuff;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.simple.SimpleMatrix;
public class Kalman
{
    // Todo: find a way to update the transitional matrices and update for a better filter
    public SimpleMatrix[] pose;
    private final SimpleMatrix STARTING_COVARIANCE = new SimpleMatrix(new double[][]
    {
            {0.1, 0, 0}, // x covariance
            {0, 0.01, 0}, // y covariance
            {0, 0, Math.toRadians(0.1)} // heading covariance
    });

    // A is something, like transitional matrix for prediction, don't this should always be an identity
    private SimpleMatrix A = SimpleMatrix.identity(3);
    // H is the sensor transitional matrix, it indicates the which values we can directly measure with the sensor
    // so if using april tage identity? and if distance sensor only x?
    //TODO: make a setter to H
    private SimpleMatrix H = SimpleMatrix.identity(3);
    private SimpleMatrix P; // system covariance


    public Kalman(SimpleMatrix[] pose)
    {
        this.pose = pose;
    }
    public Kalman(SimpleMatrix pose, SimpleMatrix cov)
    {
        this.pose = new SimpleMatrix[]{pose, cov};
    }
    public Kalman(SimpleMatrix pose)
    {
        this.pose = new SimpleMatrix[]{pose, STARTING_COVARIANCE};
    }

    // Gets the previous state(always this.pose) and the update matrix (should be the difference between previous and current, use RR)
    public SimpleMatrix[] prediction(SimpleMatrix[] prev, SimpleMatrix update)
    {
        // use this to update covariance?
        double prevX = prev[0].get(0,0);
        double prevY = prev[0].get(1,0);
        double prevHeading = prev[0].get(2,0);
        SimpleMatrix prevCov = prev[1];

        //Todo make tuning, loss of power of the system
        SimpleMatrix Q = new SimpleMatrix(new double[][]
                {
                        {0.3, 0, 0},
                        {0, 0.2, 0},
                        {0, 0, 0.9}
                });


        P = A.mult(prevCov).mult(A.transpose()).plus(Q);
        // pass the calculated values to the pose style matrix
        double predX = update.get(0, 0);
        double predY = update.get(1,0);
        double predHeading = angleWrap(update.get(2,0)); // wraps the angle ]-180, 180[

        return new SimpleMatrix[]
                {
                    new SimpleMatrix(new double[][]
                    {
                        {predX},
                        {predY},
                        {predHeading}
                    }),
                    P
                };
    }

    // Gets the prev state (always this.pose) and the matrix that is the observable state
    public SimpleMatrix[] correction(SimpleMatrix[] pred, SimpleMatrix observation)
    {
        SimpleMatrix predCov = pred[1]; // gets the predicted cov index in pose[1]

        // the state from the observation
        SimpleMatrix z = new SimpleMatrix(new double[][]
        {
            {observation.get(0,0)},
            {observation.get(1, 0)},
            {observation.get(2, 0)}
        });
        // Todo: tuning this is final value tho after tuning, so make it global?
        SimpleMatrix w = new SimpleMatrix(new double[][]
        {
            {0.4},
            {0.4},
            {0.0001}
        });

        // don't remember now what this is
        SimpleMatrix i = z.minus(H.mult(pred[0]).plus(w));;

        // Sensor noise matrix
        // TODO how to find these values? Michael said he found a sensor noise paper or something like that
        SimpleMatrix Q = new SimpleMatrix(new double[][]
                {
                        {0.20, 0, 0},
                        {0, 0.2, 0},
                        {0, 0, 0, 0.01}
                });

        // Simplified calculations
        SimpleMatrix S = H.mult(predCov).mult(H.transpose()).plus(Q);
        SimpleMatrix K = predCov.mult(H.transpose().mult(S.invert()));

        SimpleMatrix correct = pred[0].plus(K.mult(i));
        // wraps the angle
        correct.set(2, 0, angleWrap(correct.get(2,0)));
        SimpleMatrix correctCov = predCov.minus(K.mult(H).mult(predCov));
        return new SimpleMatrix[]{correct, correctCov};
    }

    // WTF?
    // Update matrix is the target - previous
    public void update(SimpleMatrix update, SimpleMatrix obs)
    {
        if (update != null && obs == null)
        {
            this.pose = prediction(pose,update);
        }
        else if (update == null && obs != null)
        {
            this.pose = correction(pose, obs);
        }
        else // if (update != null && obs != null)
        {
            // fuse does all, probably should only use this one
            this.pose = fuse(pose, update, obs);
        }
    }

    public SimpleMatrix[] fuse(SimpleMatrix[] prev, SimpleMatrix update, SimpleMatrix obs)
    {
        SimpleMatrix[] prediction = prediction(prev, update);
        return correction(prediction, obs);
    }

    public Pose2d getPose()
    {
        return new Pose2d(pose[0].get(0, 0), pose[0].get(1, 0), pose[0].get(2, 0));
    }
    public SimpleMatrix getCov()
    {
        return pose[1];
    }
    //TODO: Make a function that gets a pose2d and transforms it in a matrix
}
