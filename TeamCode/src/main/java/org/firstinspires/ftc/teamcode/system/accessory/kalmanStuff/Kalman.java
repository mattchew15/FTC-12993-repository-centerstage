package org.firstinspires.ftc.teamcode.system.accessory.kalmanStuff;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.simple.SimpleMatrix;
public class Kalman
{
/*
    MATRIX SIZES
Y = 3x1
X = 3x1
K = 3x3
H = 3x3
Q = 3x3
P = 3x3
R = 3x3
I = 3x3
*/
    public SimpleMatrix[] pose;
    private SimpleMatrix STARTING_COVARIANCE = new SimpleMatrix(new double[][]
    {
            {0.01, 0, 0}, // x covariance
            {0, 0.01, 0}, // y covariance
            {0, 0, Math.toRadians(0.01)} // heading covariance
    });

    // A is something, like transitional matrix for prediction, this should always be an identity as the prediction is always current pls update
    private SimpleMatrix A = SimpleMatrix.identity(3);
    // H is the sensor transitional matrix, it indicates the which values we can directly measure with the sensor
    // so if using april tage identity? and if distance sensor only x?=
    private SimpleMatrix H = SimpleMatrix.identity(3);
    private SimpleMatrix P; // system covarianceS
    private SimpleMatrix w = new SimpleMatrix(new double[][] // Process noise for the prediction, can it be ignored? Gonna make it 0 for now
            {
            {0.0},
            {0.0},
            {0.0}
        });
    // Sensor noise
    private SimpleMatrix R = new SimpleMatrix(new double[][]
        {
            {0.2, 0, 0},
            {0, 0.2, 0},
            {0, 0, 0.01}
        });

    // Covariance gain for the model
    private SimpleMatrix Q = new SimpleMatrix(new double[][]
        {
            {0.2, 0, 0},
            {0, 0.2, 0},
            {0, 0, 0.1}
        });


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
    public SimpleMatrix[] prediction(SimpleMatrix[] prev, SimpleMatrix update, boolean OdoOnly)
    {
        SimpleMatrix prevCov = prev[1];
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
    // Use this if using getUpdateMatrix(), it adds the update matrix to the pose
    public SimpleMatrix[] prediction(SimpleMatrix[] prev, SimpleMatrix update)
    {
        SimpleMatrix prevCov = prev[1];
        // Projects covariance
        P = (A.mult(prevCov).mult(A.transpose())).plus(Q);

        // pass the calculated values to the pose style matrix
        double predX = prev[0].get(0,0) + update.get(0, 0);
        double predY = prev[0].get(2,0) + update.get(1,0);
        double predHeading = prev[0].get(3,0) + angleWrap(update.get(2,0)); // wraps the angle ]-180, 180[

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

        // the state from the observation, needs to be local
        SimpleMatrix z = new SimpleMatrix(new double[][]
        {
            {observation.get(0,0)},
            {observation.get(1, 0)},
            {observation.get(2, 0)}
        });

        // Current difference between sensor and pred, given by sensor - (H * predicted + process noise)
        SimpleMatrix i = z.minus(H.mult(pred[0]).plus(w));;

        //bottom part of the kalman gain, use invert
        SimpleMatrix S = H.mult(predCov).mult(H.transpose()).plus(R);

        // Kalman gain
        // TODO possible math logic error, matrix division
        SimpleMatrix K = predCov.mult(H.transpose().mult(S.invert()));

        // The actual pos
        SimpleMatrix correct = pred[0].plus(K.mult(i));
        // wraps the angle
        correct.set(2, 0, angleWrap(correct.get(2,0)));
        // Updates the covariance
        // What is I on the formulas?? seems like the identity matrix but not sure anymore, was michael code correct? don't seem like, my formula should be better
        // SimpleMatrix correctCov = predCov.minus(K.mult(H).mult(predCov));
        SimpleMatrix correctCov = predCov.mult(SimpleMatrix.identity(3).minus(K.mult(H)));
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
    public static SimpleMatrix poseToMatrix(Pose2d pose2d)
    {
        return new SimpleMatrix(new double[][]
                {
                        {pose2d.getX()},
                        {pose2d.getY()},
                        {pose2d.getHeading()}
                });
    }

    /** This returns a 3x1 matrix with the first value (x), use this for the sensor stuff*/
    public SimpleMatrix doubleToMatrix(double val)
    {
        return new SimpleMatrix(new double[][]
                {
                    {val},
                    {0},
                    {0}
                });
    }

    /** H should always be an identity*/
    public void setH(double v1, double v2, double v3)
    {
        this.H = new SimpleMatrix(new double[][]
        {
            {v1, 0, 0},
            {0, v2, 0},
            {0, 0, v3}
    });
    }
    /** Use this*/
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
    public void setSTARTING_COVARIANCE(double cov1, double cov2, double cov3)
    {
         this.STARTING_COVARIANCE = new SimpleMatrix(new double[][]
                {
                        {cov1, 0, 0}, // x covariance
                        {0, cov2, 0}, // y covariance
                        {0, 0, cov3} // heading covariance
                });
    }
    public void setProcessNoise(double n1, double n2, double n3)
    {
        this.w = new SimpleMatrix(new double[][]
                {
                        {n1, 0, 0},
                        {0, n2, 0},
                        {0, 0, n3}
                });
    }
    public void setSensorNoise(double n1, double n2, double n3)
    {
        this.R = new SimpleMatrix(new double[][]
            {
                    {n1, 0, 0},
                    {0, n2, 0},
                    {0, 0, n3}
            });
    }
    public void setCovarianceGain(double c1, double c2, double c3)
    {
        this.Q = new SimpleMatrix(new double[][]
                {
                        {c1, 0, 0},
                        {0, c2, 0},
                        {0, 0, c3}
                });
    }

    /** Get the new pose2d and take of the previous pose given by the filter, this generate an update matrix */
    public SimpleMatrix getUpdateMatrix(Pose2d pose2d)
    {
        SimpleMatrix newPose = poseToMatrix(pose2d);

        return new SimpleMatrix(new double[][]
                {
                        {newPose.get(0,0) - pose[0].get(0,0)},
                        {newPose.get(1, 0) - pose[0].get(1, 0)},
                        {newPose.get(2, 0) - pose[0].get(2, 0)}
                });
    }
}
