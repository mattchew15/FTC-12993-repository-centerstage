package org.firstinspires.ftc.teamcode.system.paths.P2P.C;


import static java.lang.Math.PI;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;

@Config
public class Follower  {

    public static boolean ENABLED = true;

    public static double followingCoefficient = 1, correctionCoefficient = 0, centripetalCorrectionCoefficient = 0.006, headingPIDCoefficient = 4;

    private final MecanumDrive drive;
    private final Localizer localizer;

    private Trajectory trajectory;
    public double currentFollowedPoint = 0.001;
    public double predictedFollowedPoint = 0.001;

    private final PIDController headingPIDController = new PIDController(0,0,0);

    private boolean pid = false;

    public static PIDCoefficients correctionPIDCoefficients = new PIDCoefficients(0,0,0);
    private final PIDController correctionPID = new PIDController(0,0,0);

    public double velocityMultiplier = 1;

    public static double jerk = 8, maxAcceleration = 2;
    public VelocityAsymmetricMotionProfile velocityProfile = new VelocityAsymmetricMotionProfile(maxAcceleration, jerk, jerk);

    public Follower(MecanumDrive drive){
        this.drive = drive;
        this.localizer = drive.getLocalizer();
    }

    public void setTrajectory(Trajectory newTrajectory){
        setTrajectory(newTrajectory, 1);
    }

    public void setTrajectory(Trajectory newTrajectory, double maxPower){
        this.trajectory = newTrajectory;
        this.pid = false;
        this.drive.setRunMode(MecanumDrive.RunMode.Vector);
        this.currentFollowedPoint = 0.001;
        this.velocityMultiplier = maxPower;
        velocityProfile.setMotion(0,1,0);
    }

    public Trajectory getTrajectory(){
        return trajectory;
    }

    public Vector tangentVelocityVector = new Vector();
    public Vector predictedTangentVelocityVector = new Vector();
    public Vector driveVector = new Vector();
    public Vector correctingVector = new Vector();

    public double velocity = 0;

    public void update() {
        if(!ENABLED) return;

        if(trajectory == null) return;
        if(pid) return;

        if(trajectory.getLength() - trajectory.getLengthAt(currentFollowedPoint) <= drive.getLocalizer().glideDelta.getMagnitude()){
            pid = true;
            drive.setRunMode(MecanumDrive.RunMode.PID);
            drive.setTargetPose(trajectory.getPose(1));
        }

        velocity = localizer.getVelocity().getMagnitude();

        Pose currentPose = localizer.getPoseEstimate();

        Pose predictedPose = localizer.getPredictedPoseEstimate();

        currentFollowedPoint = trajectory.getFollowedPoint(currentPose, currentFollowedPoint);
        predictedFollowedPoint = trajectory.getFollowedPoint(predictedPose, currentFollowedPoint);

        tangentVelocityVector = trajectory.getTangentVelocity(currentFollowedPoint)
                .scaledToMagnitude(velocityProfile.getPosition())
//                .scaledToMagnitude(1)
                .scaledBy(velocityMultiplier)
                .scaledBy(followingCoefficient);
        predictedTangentVelocityVector = trajectory.getTangentVelocity(predictedFollowedPoint).scaledBy(followingCoefficient);

        Pose trajectoryPose = trajectory.getPose(currentFollowedPoint);

        correctionPIDCoefficients = new PIDCoefficients(MecanumDrive.TRANSLATIONAL_PID.getP(), MecanumDrive.TRANSLATIONAL_PID.getI(), MecanumDrive.TRANSLATIONAL_PID.getD());
        correctionPID.setPID(correctionPIDCoefficients.p, correctionPIDCoefficients.i, correctionPIDCoefficients.d);

        correctingVector = new Vector(trajectoryPose.getX() - currentPose.getX(), trajectoryPose.getY() - currentPose.getY(), 0);
        double correctionPower = correctionPID.calculate(-correctingVector.getMagnitude(),0);
        correctingVector.scaleToMagnitude(correctionPower);
        correctingVector.scaleBy(correctionCoefficient);

        Vector centripetalCorrectionVector = new Vector(Math.cos(Math.atan2(predictedTangentVelocityVector.getY(), predictedTangentVelocityVector.getX()) + PI/2.0),
                Math.sin(Math.atan2(predictedTangentVelocityVector.getY(), predictedTangentVelocityVector.getX()) + PI/2.0))
                .scaledBy(trajectory.getCurvature(predictedFollowedPoint) * centripetalCorrectionCoefficient)
                .scaledBy(velocity * velocity);


        headingPIDController.setPID(MecanumDrive.HEADING_PID.getP(), MecanumDrive.HEADING_PID.getI(), MecanumDrive.TRANSLATIONAL_PID.getD());
        double headingDelta = trajectoryPose.getHeading() - currentPose.getHeading();
        headingDelta%=2.0 * PI;
        if(headingDelta > PI) headingDelta -= 2.0*PI;
        if(headingDelta < -PI) headingDelta += 2.0*PI;

        Vector turningVector = new Vector(0, 0, headingPIDController.calculate(-headingDelta, 0)).scaledBy(headingPIDCoefficient);

        driveVector = tangentVelocityVector.plus(correctingVector).plus(centripetalCorrectionVector);
        driveVector.scaleToMagnitude(1);
        driveVector = driveVector.plus(turningVector);

        drive.setTargetVector(driveVector);
        velocityProfile.update();
    }

}