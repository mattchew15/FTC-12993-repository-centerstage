package org.firstinspires.ftc.teamcode.system.accessory.kalmanStuff.Michael;
import static org.firstinspires.ftc.teamcode.system.accessory.kalmanStuff.Michael.Relocalizer.Side.BLUE;
import static org.firstinspires.ftc.teamcode.system.accessory.kalmanStuff.Michael.Relocalizer.Side.RED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Michael.util.Better2MDriver;
// TODO: what this object does

//relocalizes the robot in the warehouse based on distance sensor angles and alliance color
public class Relocalizer {
    Better2MDriver distF;
    Better2MDriver distR;
    Better2MDriver distL;

    //1 is front, 2 is right, 3 is left
    double angle1=0, angle2=Math.toRadians(-90), angle3=Math.toRadians(90);
    public double angle=0; private double angleOffset = 0;
    private boolean[] outOfRange = {false, false , false};
    Pose2d oldPose= new Pose2d(0,0,0);
    Pose2d[] sensors={new Pose2d(8.5,4.8,Math.toRadians(0)), new Pose2d(6.1,-4.35,Math.toRadians(-90)), new Pose2d(6.1,4.35,Math.toRadians(90))};
    private final double[] offset = {59.036, 9.33, 54.507, 7.492};
    public enum Side {
        RED, BLUE
    }
    Side side;
    public DistanceSensor dist1;
    public DistanceSensor dist2;
    public DistanceSensor dist3;
    //public AsyncMB1242 distF;

    BNO055IMU imu;

    public Relocalizer(HardwareMap hardwareMap, Side side) {
        //if (BaseAutonomous.autorunning)
        dist1 = hardwareMap.get(DistanceSensor.class, "distF");
        //distF = hardwareMap.get(AsyncMB1242.class, "distFL");
        dist2 = hardwareMap.get(DistanceSensor.class, "distR");
        dist3 = hardwareMap.get(DistanceSensor.class, "distL");

        distF = new Better2MDriver(dist1);
        distR = new Better2MDriver(dist2);
        distL = new Better2MDriver(dist3);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        /*telemetry.addData("Distance", asyncSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Last Reading", asyncSensor.getLastMeasurementTimestamp());

            if(gamepad1.a){
                asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_ACCURACY);*/
        this.side = side;
    }
    public void setAngleEstimate(double angle) {
        angleOffset = angle;
    }
    public double getAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;//+angleOffset;
    }

    public void setSide (Side side){
        this.side=side;
    }

    public void setSide (boolean side){ //TODO: Make this a get our auto side enum
        if (side) {
            setSide(BLUE);
        } else {
            setSide(RED);
        }
    }
    /*
        public Pose2d relocalize(int sidestuff, Pose2d estimatedPose){
            double anglewrapped = angleWrap(Math.toDegrees(getAngle()));
            //i was gonna make this more complicated but i want to keep it simple for now, so just assume the distance sensors are both facing the correct walls
            if (sidestuff == 1) {
                angle = anglewrapped+Math.toRadians(180);
            } else if (sidestuff == 2) {
                angle = getAngle();
            } else {
                return new Pose2d(999, 0, 0);
            }
            if (sidestuff !=3) {
                double dist1 = this.distF.getDistance(DistanceUnit.INCH);///Math.cos(Math.toRadians(20));

                if (dist1>100) return new Pose2d(999,0,0);

                double dist2,dist3;
                double x=Math.cos(angle+angle1)*dist1;
                double xOff=sensors[0].vec().rotated(angle).getX();
                double y;
                boolean wall = true; //true = red
                if (anglewrapped%90>60) {
                    wall = true;
                } else if (anglewrapped%90<30) {
                    wall = false;
                }
                double yOff;
                if(wall){
                    dist2 = this.distR.getDistance(DistanceUnit.INCH);
                    if (dist2>100) return new Pose2d(999,0,0);

                    y=-Math.cos(Math.toRadians(90)+(angle2+angle))*dist2;
                    yOff=sensors[1].vec().rotated(angle).getY();
                } else{
                    dist3 = this.distL.getDistance(DistanceUnit.INCH);
                    if (dist3>100) return new Pose2d(999,0,0);

                    y=Math.cos(Math.toRadians(90)-(angle3+angle))*dist3;
                    yOff=sensors[2].vec().rotated(angle).getY();
                }
                Pose2d pose=new Pose2d(x,y,Math.toRadians(anglewrapped));
                if(side == RED){
                    pose=new Pose2d(72-pose.getX(),-72-pose.getY(), pose.getHeading());
                }else{
                    pose=new Pose2d(72-pose.getX(),72-pose.getY(), pose.getHeading());
                }
                oldPose=pose;
                int stuff = quadrant(estimatedPose);
                if (stuff == 2) {
                    pose = new Pose2d(-pose.getY(), pose.getX(), anglewrapped);
                    if (!wall) {
                        pose = new Pose2d(pose.getX()+xOff, pose.getY()+yOff, pose.getHeading());
                    } else {
                        pose = new Pose2d(pose.getX()-xOff, pose.getY()-yOff, pose.getHeading());
                    }
                } else if (stuff == 3) {
                    pose = new Pose2d(-(144-pose.getX()), -(144+pose.getY()), anglewrapped);
                    if (wall) {
                        pose = new Pose2d(pose.getX()+yOff, pose.getY()+xOff, pose.getHeading());
                    } else {
                        pose = new Pose2d(pose.getX()-yOff, pose.getY()-xOff, pose.getHeading());
                    }
                } else if (stuff == 4) {
                    pose = new Pose2d(pose.getY(), -pose.getX(), anglewrapped);
                    if (!wall) {
                        pose = new Pose2d(pose.getX()+xOff, pose.getY()+yOff, pose.getHeading());
                    } else {
                        pose = new Pose2d(pose.getX()-xOff, pose.getY()-yOff, pose.getHeading());
                    }
                } else {
                    //cool!
                    if (wall) {
                        pose = new Pose2d(pose.getX()+yOff, pose.getY()+xOff, pose.getHeading());
                    } else {
                        pose = new Pose2d(pose.getX()-yOff, pose.getY()-xOff, pose.getHeading());
                    }
                }
                return pose;
            }
            return new Pose2d(999, 0, 0 );
        }
    */
    public Pose2d relocalize(){
        boolean F = true, L = true, R = true, S = true, b = true;
        if (distF.getDistance(DistanceUnit.INCH) > 100) F = false;
        if (distL.getDistance(DistanceUnit.INCH) > 100) L = false;
        if (distR.getDistance(DistanceUnit.INCH) > 100) R = false;
        if (F && L) b = true;
        if (F && R) b = false;
        if (L && R) S = false;
        //i was gonna make this more complicated but i want to keep it simple for now, so just assume the distance sensors are both facing the correct walls
        if (F && S) {
            angle=getAngle()+((side==BLUE)?Math.toRadians(180):0);
            double dist1 = this.distF.getDistance(DistanceUnit.INCH);///Math.cos(Math.toRadians(20));

            if (dist1>100) return new Pose2d(999,0,0);

            double dist2,dist3;
            double x=Math.cos(angle+angle1)*dist1;
            double xOff=sensors[0].vec().rotated(angle).getX();
            x+=xOff;
            double y;
            double transformedH = angle%90;
            if(b){
                dist2 = this.distR.getDistance(DistanceUnit.INCH);
                if (dist2>100) return new Pose2d(999,0,0);

                y=-Math.cos(Math.toRadians(90)+(angle2+transformedH))*dist2;
                double yOff=sensors[1].vec().rotated(transformedH).getY();
                y-=yOff;
            }
            else{
                dist3 = this.distL.getDistance(DistanceUnit.INCH);
                if (dist3>100) return new Pose2d(999,0,0);

                y=Math.cos(Math.toRadians(90)-(angle3+transformedH))*dist3;
                double yOff=sensors[2].vec().rotated(transformedH).getY();
                y+=yOff;
            }
            Pose2d pose=new Pose2d(x,y,transformedH);
            if(b){
                pose=new Pose2d(72-pose.getX(),-72-pose.getY(), pose.getHeading());
            }else{
                pose=new Pose2d(72-pose.getX(),72-pose.getY(), pose.getHeading());
            }
            double anglewrapped = angleWrap(angle);
            if (anglewrapped>90 && anglewrapped<=180) {
                pose = new Pose2d(-pose.getY(), pose.getX(), anglewrapped);
            } else if (anglewrapped<0 && anglewrapped>=-90) {
                pose = new Pose2d(pose.getY(), -pose.getX(), anglewrapped);
            } else if (anglewrapped<-90 && anglewrapped>-180) {
                pose = new Pose2d(-pose.getX(), -pose.getY(), anglewrapped);
            }
            oldPose=pose;
            return pose;
        } else {
            return new Pose2d(999, 0, 0);
        }

    }
    public int quadrant(Pose2d pose) {
        if (pose.getX()<0 && pose.getY()>0) {
            return 2;
        } else if (pose.getX()<0 && pose.getY()<0) {
            return 3;
        } else if (pose.getX()>0 && pose.getY()<0) {
            return 4;
        } else {
            return 1;
        }
    }
    public double angleWrap(double input) {
        input%=360;
        if (input>180) input-=360;
        return input;
    }
    //get distF
    public double getDistF(){
        return distF.getDistance(DistanceUnit.INCH);
    }

    //get distR
    public double getDistR(){
        return distR.getDistance(DistanceUnit.INCH);
    }

    //get distL
    public double getDistL(){
        return distL.getDistance(DistanceUnit.INCH);
    }
}