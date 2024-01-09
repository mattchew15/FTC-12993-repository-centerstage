package org.firstinspires.ftc.teamcode.system.paths.PurePersuit;

import androidx.core.math.MathUtils;


public class Robot {
    public static boolean usingComputer = true;

    /**
     * Creates a robot simulation
     */
    public Robot(){
        worldXPosition = 150;
        worldYPosition = 0;
        worldAngle_rad = Math.toRadians(90);
    }

    //the actual speed the robot is moving
    public static double xSpeed = 0;
    public static double ySpeed = 0;
    public static double turnSpeed = 0;

    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;

    public double getXPos(){
        return worldXPosition;
    }

    public double getYPos(){
        return worldYPosition;
    }


    public double getWorldAngle_rad() {
        return worldAngle_rad;
    }


    //last update time
    private long lastUpdateTime = 0;

    /**
     * Calculates the change in position of the robot
     */
    public void update(){
        //get the current time
        long currentTimeMillis = System.currentTimeMillis();
        //get the elapsed time
        double elapsedTime = (currentTimeMillis - lastUpdateTime)/1000.0;
        //remember the lastUpdateTime
        lastUpdateTime = currentTimeMillis;
        if(elapsedTime > 1){return;}



        //increment the positions
        double totalSpeed = Math.hypot(xSpeed,ySpeed);
        double angle = Math.atan2(ySpeed,xSpeed) - Math.toRadians(90);
        double outputAngle = worldAngle_rad + angle;
        worldXPosition += totalSpeed * Math.cos(outputAngle) * elapsedTime * 1000 * 0.2;
        worldYPosition += totalSpeed * Math.sin(outputAngle) * elapsedTime * 1000 * 0.2;

        worldAngle_rad += MovementVars.movement_turn * elapsedTime * 20 / (2 * Math.PI);


        xSpeed += MathUtils.clamp((MovementVars.movement_x-xSpeed)/0.2,-1,1) * elapsedTime;
        ySpeed += MathUtils.clamp((MovementVars.movement_y-ySpeed)/0.2,-1,1) * elapsedTime;
        turnSpeed += MathUtils.clamp((MovementVars.movement_turn-turnSpeed)/0.2,-1,1) * elapsedTime;


//        SpeedOmeter.yDistTraveled += ySpeed * elapsedTime * 1000;
//        SpeedOmeter.xDistTraveled += xSpeed * elapsedTime * 1000;

//        SpeedOmeter.update();


        xSpeed *= 1.0 - (elapsedTime);
        ySpeed *= 1.0 - (elapsedTime);
        turnSpeed *= 1.0 - (elapsedTime);



    }
}
