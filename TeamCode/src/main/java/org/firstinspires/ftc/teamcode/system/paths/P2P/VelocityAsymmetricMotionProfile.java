package org.firstinspires.ftc.teamcode.system.paths.P2P;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VelocityAsymmetricMotionProfile {
    private final double acceleration, deceleration;
    private final double maxVelocity;
    private double initialVelocity;
    public double initialPosition, finalPosition;
    private double deltaPose, sign;
    private double maxReachedVelocity;
    private double t0,t1,t2,t3,t;
    private double v0;

    private double position, velocity, signedVelocity;

    public final ElapsedTime timer = new ElapsedTime();

    public VelocityAsymmetricMotionProfile(double maxVelocity, double acceleration, double deceleration){
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.deceleration = deceleration;

        setMotion(0,0,0);
    }

    public void setMotion(double initialPosition, double finalPosition, double initialVelocity){
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.initialVelocity = Math.signum(initialVelocity) * Math.min(Math.abs(initialVelocity), maxVelocity);
        sign = Math.signum(finalPosition - initialPosition);
        v0 = sign * initialVelocity;
        t0 = v0/acceleration;
        deltaPose = t0*v0/2.0 + Math.abs(finalPosition - initialPosition);
        maxReachedVelocity = Math.max((calculateDeltaIfMaxReachedVelocityIs(maxVelocity) - deltaPose) <= 0 ? maxVelocity :
                Math.sqrt(deltaPose*2.0*acceleration*deceleration/(acceleration+deceleration)), v0);
        t1 = maxReachedVelocity/acceleration - t0;
        t3 = maxReachedVelocity/deceleration;
        t2 = Math.abs(Math.min(0, calculateDeltaIfMaxReachedVelocityIs(maxVelocity)-deltaPose))/maxVelocity;
        t=t1+t2+t3;
        timer.reset();
    }

    private double calculateDeltaIfMaxReachedVelocityIs(double v){
        return (v*v/2.0)*(acceleration+deceleration)/(acceleration*deceleration);
    }

    public int getPhase(){
        if(timer.seconds() <= t1) return 1;
        if(timer.seconds() <= t1+t2) return 2;
        if(timer.seconds() <= t1+t2+t3) return 3;
        return 0;
    }

    private double getVelocity(int phase, double time){
        switch (phase){
            case 1: return v0+time*acceleration;
            case 2: return getVelocity(1, t1);
            case 3: return maxReachedVelocity - deceleration * (time - t1 - t2);
        }
        return 0;
    }

    public double getPosition(double time){
        if(time <= t1) return initialPosition + sign*v0*time/2.0 + sign*getVelocity(1,time)*time/2.0;
        if(time <= t1+t2) return initialPosition + sign*v0*t1/2.0 + sign*getVelocity(1,t1)*t1/2.0 + sign*getVelocity(2, time)*(time-t1);
        if(time <= t1+t2+t3) return getPosition(t1+t2) + sign*maxReachedVelocity*t3/2.0 - sign*getVelocity(3, time) * (t1+t2+t3-time)/2.0;
        return finalPosition;
    }

    private void updateVelocity(){
        this.velocity = getVelocity(getPhase(), timer.seconds());
    }

    private void updateSignedVelocity(){
        this.signedVelocity = sign * getVelocity(getPhase(), timer.seconds());
    }

    private void updatePosition(){
        this.position = getPosition(timer.seconds());
    }

    public double getVelocity(){
        return velocity;
    }

    public double getSignedVelocity(){
        return signedVelocity;
    }

    public double getPosition(){
        return position;
    }

    public double getTimeToMotionEnd(){
        return Math.max(0,t - timer.seconds());
    }

    private double getTime1(double position) {
        return (-sign * v0 - Math.signum(v0) * Math.sqrt(v0 * v0 - 2 * sign * acceleration*(initialPosition-position))) / (sign * acceleration);
    }

    private double getTime2(double position) {
        return (position - getPosition(t1)) / (sign * (v0 + t1 * acceleration)) + t1;
    }

    private double getTime3(double position) {
        double qa = -deceleration;
        double qb = deceleration * (t + t1 + t2) + maxReachedVelocity;
        double qc = (2.0 / sign) * (getPosition(t1 + t2) - position) + maxReachedVelocity * t3 - t * (maxReachedVelocity + deceleration * (t1 + t2));
        return (-qb + Math.sqrt(qb * qb - 4.0 * qa * qc)) / (2.0 * qa);
    }

    public double getTime(double position) {
        if (position >= Math.min(getPosition(t1), getPosition(t1 + t2)) && position <= Math.max(getPosition(t1), getPosition(t1 + t2)))
            return getTime2(position);

        double s = getTime1(position);
        if (s < 0) s = 0;
        if (s > t1) {
            s = getTime3(position);
            if (s > t1 + t2 + t3)
                s = t1 + t2 + t3;
        }
        return s;
    }

    public double getTimeTo(double position){
        return Math.max(0,getTime(position) - timer.seconds());
    }

    public void telemetry(Telemetry telemetry){
        telemetry.addData("Profile initial position", initialPosition);
        telemetry.addData("Profile final position", finalPosition);
        telemetry.addData("Profile initial velocity", initialVelocity);
        telemetry.addData("Profile current velocity", velocity);
        telemetry.addData("Plateau distance", calculateDeltaIfMaxReachedVelocityIs(maxVelocity));
        telemetry.addData("Max Velocity", maxVelocity);
        telemetry.addData("Max reached Velocity", maxReachedVelocity);
        telemetry.addData("Acceleration", acceleration);
        telemetry.addData("Deceleration", deceleration);
        telemetry.addData("Delta pose", deltaPose);
        telemetry.addData("t1", t1);
        telemetry.addData("t2", t2);
        telemetry.addData("t3", t3);
        telemetry.addData("Phase", getPhase());
        telemetry.addData("Time to motion end", getTimeToMotionEnd());
    }

    public void update(){
        updateVelocity();
        updateSignedVelocity();
        updatePosition();
    }
}