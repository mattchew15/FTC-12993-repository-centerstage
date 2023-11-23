package org.firstinspires.ftc.teamcode.system.accessory.kalmanStuff;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.simple.SimpleMatrix;
public class Kalman
{
    //TODO: Check math
    public SimpleMatrix[] pose;
    private final SimpleMatrix STARTING_COVARIANCE = new SimpleMatrix(new double[][]
    {
            {0.1, 0, 0}, // x covariance
            {0, 0.01, 0}, // y covariance
            {0, 0, Math.toRadians(0.1)} // heading covariance
    });

    // A is something, like transitional matrix for prediction, this should always be an identity as the prediction is always current pls update
    private SimpleMatrix A = SimpleMatrix.identity(3);
    // H is the sensor transitional matrix, it indicates the which values we can directly measure with the sensor
    // so if using april tage identity? and if distance sensor only x?=
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

    // Gets the previous state(always this.pose) and the update matrix should be just RR value; this to work it needs a change (should be the difference between previous and current, use RR)
    public SimpleMatrix[] prediction(SimpleMatrix[] prev, SimpleMatrix update)
    {
        SimpleMatrix prevCov = prev[1];

        //Todo make setter, loss of power of the system, make this global possibly final
        //Covariance gain for the model
        SimpleMatrix Q = new SimpleMatrix(new double[][]
                {
                        {0.3, 0, 0},
                        {0, 0.2, 0},
                        {0, 0, 0.9}
                });
        // Projects covariance
        P = (A.mult(prevCov).mult(A.transpose())).plus(Q);

        // pass the calculated values to the pose style matrix
        double predX = update.get(0, 0);
        double predY = update.get(1,0);
        double predHeading = angleWrap(update.get(2,0)); // wraps the angle ]-180, 180[
        // Return a matrix on the pose style
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

    // Gets the pred state (always this.pose, take out the argument?) and the matrix that is the observable state
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
        // Process noise for the prediction, can it be ignored? prob not
        SimpleMatrix w = new SimpleMatrix(new double[][]
        {
            {0.4},
            {0.4},
            {0.0001}
        });

        // Current difference between sensor and pred, given by sensor - (predicted + process noise)
        SimpleMatrix i = z.minus(H.mult(pred[0]).plus(w));;

        // Sensor noise
        // TODO how to find these values? Michael said he found a sensor noise paper or something like that
        SimpleMatrix R = new SimpleMatrix(new double[][]
                {
                        {0.20, 0, 0},
                        {0, 0.2, 0},
                        {0, 0, 0, 0.01}
                });

        //bottom part of the kalman gain, use invert
        SimpleMatrix S = H.mult(predCov).mult(H.transpose()).plus(R);
        // Kalman gain
        SimpleMatrix K = predCov.mult(H.transpose().mult(S.invert()));
        // The actual pos
        SimpleMatrix correct = pred[0].plus(K.mult(i));
        // wraps the angle
        correct.set(2, 0, angleWrap(correct.get(2,0)));
        // Updates the covariance
        // What is I on the formulas?? seems like the identity matrix but not sure anymore, was michael code correct? don't seem like, my formula should be better
        // SimpleMatrix correctCov = predCov.minus(K.mult(H).mult(predCov));
        SimpleMatrix correctCov = predCov.mult(SimpleMatrix.identity(3).minus(K).mult(H));
        return new SimpleMatrix[]{correct, correctCov};
    }

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
        SimpleMatrix[] prediction = prediction(prev, update); // this is my new pose isn't it?
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

    /** This return a 3x1 matrix, use this for the all the matrix and stuff*/
    public SimpleMatrix poseToMatrix(Pose2d pose2d)
    {
        SimpleMatrix matrix = new SimpleMatrix(new double[][]
                {
                    {pose2d.getX()},
                    {pose2d.getY()},
                    {pose2d.getHeading()}
                });
        return matrix;
    }
    /** This returns a 3x1 matrix with the first value (x), use this for the sensor stuff*/
    public SimpleMatrix doubleToMatrix(double val)
    {
        SimpleMatrix matrix = new SimpleMatrix(new double[][]
                {
                        {val},
                        {0},
                        {0}
                });
        return matrix;
    }

    public void setH(double v1, double v2, double v3)
    {
        this.H = new SimpleMatrix(new double[][]
        {
            {v1, 0, 0},
            {0, v2, 0},
            {0, 0, v3}
    });
    }
    public void setH(boolean all)
    {
        if (all)
        {
            this.H = SimpleMatrix.identity(3);
        }
        else
        {
            this.H = new SimpleMatrix(new double[][]
                {
                        // 3x1 or 3x3?
                        {1},
                        {0},
                        {0}
                });
        }
    }

}
