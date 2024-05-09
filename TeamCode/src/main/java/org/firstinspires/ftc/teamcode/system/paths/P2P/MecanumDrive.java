package org.firstinspires.ftc.teamcode.system.paths.P2P;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.angleWrap;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.AngleWrap;

import android.os.LocaleList;

import androidx.core.math.MathUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PeriodicSupplier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.PID;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;

public class MecanumDrive
{
    public static boolean ENABLED = true;
    public enum RunMode
    {
        PID,
        P2P,
        Vector,
        PP

    }

    private PID TRANSLATIONAL_PID = new PID(0.016, 0, 0.00000, 0, 0);
    private PID HEADING_PID = new PID(0.6, 0.000, 0.024, 312, 0);
    private DcMotor FL, FR, BL, BR;
    private RunMode runMode;
    private Localizer localizer;
    public Vector powerVector = new Vector();
    private Pose targetPose = new Pose();
    public Vector targetVector = new Vector();

    public static double ks = 0.03;
    private double lateralMultiplier;
    private double headingMultiplier;
    private double overallMultiplier;

    private final double velocityThreshold = 1;

    private TimedSupplier<Double> voltageSupplier;

    public MecanumDrive(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR, RunMode runMode, TimedSupplier<Double> supplier)
    {
        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;

        this.runMode = runMode;

        this.voltageSupplier = supplier;
    }

    private void updatePowerVector()
    {
        switch (runMode)
        {

            case PID:
                // This is gonna be a standard pid
               // driveToPosition(0, 0, 0, null);
                break;
            case P2P:
                P2P();
                break;
            case Vector:
                powerVector = new Vector(targetVector);
                powerVector = Vector.rotateBy(powerVector, localizer.getHeading());
                powerVector = new Vector(powerVector.getX(), powerVector.getY() * lateralMultiplier, targetVector.getZ());
                break;
            case PP:
                break;
        }
        if (Math.abs(powerVector.getX()) + Math.abs(powerVector.getY()) + Math.abs(powerVector.getZ()) > 1)
            powerVector.scaleToMagnitude(1);
        powerVector.scaleBy(overallMultiplier);
    }

    private void driveToPosition(double targetX, double targetY, double targetHeading, Pose2d poseEstimate)
    {
        double robotX = poseEstimate.getX();
        double robotY = poseEstimate.getY();
        double robotTheta = poseEstimate.getHeading();
        double x = TRANSLATIONAL_PID.update(targetX, robotX, 1);
        double y = -TRANSLATIONAL_PID.update(targetY, robotY, 1);
        double theta = -HEADING_PID.updateWithError(AngleWrap(targetHeading - robotTheta), 1);


        double x_rotated = (x * Math.cos(robotTheta) - y * Math.sin(robotTheta));
        double y_rotated = (x * Math.sin(robotTheta) + y * Math.cos(robotTheta));

        double FL = MathUtils.clamp(x_rotated + y_rotated + theta, -1, 1);
        double BL = MathUtils.clamp(x_rotated - y_rotated + theta, -1, 1);
        double FR = MathUtils.clamp(x_rotated - y_rotated - theta, -1, 1);
        double BR = MathUtils.clamp(x_rotated + y_rotated - theta, -1, 1);


        this.FL.setPower(FL);
        this.BL.setPower(BL);
        this.FR.setPower(FR);
        this.BR.setPower(BR);
    }

    private void P2P()
    {
        Pose currentPose = localizer.getPredictedPoseEstimate();

        double xDiff = targetPose.getX() - currentPose.getX();
        double yDiff = targetPose.getY() - currentPose.getY();

        double distance = Math.hypot(xDiff, yDiff);

        double calculatedCos = xDiff / distance;
        double calculatedSin = yDiff / distance;

        double translationalPower = TRANSLATIONAL_PID.update(-distance, 0, Double.POSITIVE_INFINITY);

        powerVector = new Vector(translationalPower * calculatedCos, translationalPower * calculatedSin);
        powerVector = powerVector.rotated(currentPose.getHeading());

        double headingDiff = angleWrap(targetPose.getHeading() - currentPose.getHeading());

        double headingPower = HEADING_PID.update(-headingDiff, 0, 1) * headingMultiplier;

        powerVector = new Vector(powerVector.getX(), powerVector.getY() * lateralMultiplier, headingPower);
    }

    private void updateMotors()
    {
        if (runMode != RunMode.PID && runMode != RunMode.PP)
        {
            double actualKs = ks * 12.0 / voltageSupplier.get();
            // This doesn't make sense like FL is +++ and FR ++-
            FL.setPower((powerVector.getX() - powerVector.getY() - powerVector.getZ()) * (1 - actualKs)
                    + actualKs * Math.signum(powerVector.getX() - powerVector.getY() - powerVector.getZ()));

            FR.setPower((powerVector.getX() + powerVector.getY() + powerVector.getZ()) * (1 - actualKs)
                    + actualKs * Math.signum(powerVector.getX() + powerVector.getY() + powerVector.getZ()));

            BL.setPower((powerVector.getX() + powerVector.getY() - powerVector.getZ()) * (1 - actualKs)
                    + actualKs * Math.signum(powerVector.getX() + powerVector.getY() - powerVector.getZ()));

            BR.setPower((powerVector.getX() - powerVector.getY() + powerVector.getZ()) * (1 - actualKs)
                    + actualKs * Math.signum(powerVector.getX() - powerVector.getY() + powerVector.getZ()));
        } else
        {
            //
        }
    }

    public void update()
    {
        if(!ENABLED) return;
        localizer.update();
        //TODO maybe have to assign an interface on this
        updatePowerVector();
        updateMotors();
    }


    public void setTargetPose(Pose pose){
        this.targetPose = pose;
    }

    public void setTargetVector(Vector Vector){
        this.targetVector = Vector;
    }

    public RunMode getRunMode() {
        return runMode;
    }

    public Localizer getLocalizer(){
        return localizer;
    }
    public void setLocalizer(Localizer localizer){
        this.localizer = localizer;
    }

    public Pose getTargetPose(){
        return targetPose;
    }

    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }

    public boolean reachedTarget(double tolerance){
        if(runMode == RunMode.Vector) return false;
        return localizer.getPoseEstimate().getDistance(targetPose) <= tolerance;
    }

    public boolean reachedHeading(double tolerance){
        if(runMode == RunMode.Vector) return false;
        return Math.abs(angleWrap(targetPose.getHeading() - localizer.getHeading())) <= tolerance;
    }

    public boolean stopped(){
        return localizer.getVelocity().getMagnitude() <= velocityThreshold;
    }




}
