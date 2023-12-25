package debugclient.src.com.company;

import java.util.ArrayList;

public class MessageProcessing {
    public MessageProcessing(){

    }

    public void processMessage(String receivedMessage) {
        System.out.println("RECEIVED: " + receivedMessage);


        //first there might be multiple messages in one packet. These are separated by '%'
        String[] splitMessages = receivedMessage.split("%");


        //go through all the messages now
        for(String message : splitMessages){
            //the individual messages are split using commas
            String[] splitString = message.split(",");
            String id = splitString[0];

            if(id.equals("ROBOT")){
                System.out.println("updating robot");
                processRobotLocation(splitString);
            }else{
                if(id.equals("P")){//POINT codes for debug point, just display it on the screen as a dot
                    processPoint(splitString);
                }else{
                    if(id.equals("LINE")){
                        System.out.println("updating line");

                        processLine(splitString);
                    }else{
                        if(id.equals("LP")){//log point
                            //add point
                            addPoint(splitString);
                        }else{
                            if(id.equals("CLEARLOG")){
                                clearLogPoints();
                            }else{
                                if(id.length() >= 5){
                                    if(id.substring(0,5).equals("CLEAR")){
                                        System.out.println("clearing");
                                        clear();
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * Clears all the log points
     */
    private void clearLogPoints() {
        pointLog.clear();
    }


    private static double robotX = 0;
    private static double robotY = 0;
    private static double robotAngle = 0;

    public static double getRobotX() { return robotX; }
    public static double getRobotY() { return robotY; }
    public static double getRobotAngle() {
        return robotAngle;
    }


    public static double getInterpolatedRobotX(){
        long currTime = System.currentTimeMillis();
        long elapsedSinceLast = currTime - lastClearTime;
        double distanceToCover = robotX - lastRobotX;

        return lastRobotX + (distanceToCover * ((double) elapsedSinceLast/lastElapsedTime));
    }

    public static double getInterpolatedRobotY(){
        long currTime = System.currentTimeMillis();
        long elapsedSinceLast = currTime - lastClearTime;
        double distanceToCover = robotY - lastRobotY;

        return lastRobotY + (distanceToCover * ((double) elapsedSinceLast/50));
    }

    public static double getInterpolatedRobotAngle(){
        long currTime = System.currentTimeMillis();
        long elapsedSinceLast = currTime - lastClearTime;
        double distanceToCover = MyMath.AngleWrap(robotAngle - lastRobotAngle);

        return lastRobotAngle + (distanceToCover * ((double) elapsedSinceLast/50));
    }



    private static double lastRobotX = 0;
    private static double lastRobotY = 0;
    private static double lastRobotAngle = 0;


    private static long lastTimeUpdate = 0;
    private static long lastElapsedTime = 0;


    /**
     * This processes the robot location and saves it's position
     * @param splitString
     */
    private void processRobotLocation(String[] splitString) {
        if(splitString.length != 4){return;}
        lastRobotX = robotX;
        lastRobotY = robotY;
        lastRobotAngle = robotAngle;


        robotX = Double.parseDouble(splitString[1]);
        robotY = Double.parseDouble(splitString[2]);
        robotAngle = Double.parseDouble(splitString[3]);

        lastElapsedTime = System.currentTimeMillis()-lastTimeUpdate;

        lastTimeUpdate = System.currentTimeMillis();
    }






    //this handles the list of debugPoints to be drawn on the screen
    ArrayList<floatPoint> debugPoints = new ArrayList<>();

    //this is used to show paths
    public static ArrayList<floatPoint> pointLog = new ArrayList<>();

    ArrayList<Line> debugLines = new ArrayList<>();

    /**
     * Takes a String[] and parses it into a point, adding it to the list of display points.
     * @param splitString
     */
    private void processPoint(String[] splitString) {
        if(splitString.length != 3){return;}
        debugPoints.add(new floatPoint(Double.parseDouble(splitString[1]),Double.parseDouble(splitString[2])));
    }





    /**
     * Adds to the list of point log
     * @param splitString String[] to be parsed into a point
     */
    private void addPoint(String[] splitString) {
        if(splitString.length != 3){return;}
        floatPoint toBeAddedMaybe = new floatPoint(Double.parseDouble(splitString[1]),
                Double.parseDouble(splitString[2]));
        //make sure the point doesn't already exist (close enough) in the list
        boolean alreadyExists = false;
        for(floatPoint p : pointLog){
            if(Math.hypot(p.x-toBeAddedMaybe.x,p.y-toBeAddedMaybe.y) < 1.5){
                alreadyExists = true;
            }
        }
        //add it if it's unique
        if(!alreadyExists){
            pointLog.add(toBeAddedMaybe);
        }
    }


    private void processLine(String[] splitString) {
        if(splitString.length != 5){return;}
        debugLines.add(new Line(Double.parseDouble(splitString[1]),
                Double.parseDouble(splitString[2]),
                Double.parseDouble(splitString[3]),
                Double.parseDouble(splitString[4])));
    }



    private static long lastClearTime = 0;
    private static double elapsedMillisThisUpdate = 0;
    /**
     * @return the elapsed time between the last two clears in milliseconds
     */
    public static double getElapsedMillisThisUpdate() {
        return elapsedMillisThisUpdate;
    }
    /**
     * Gets the time of the last clear
     * @return
     */
    public static double getLastClearTime(){
        return lastClearTime;
    }






    /**
     * Clears the debug points ArrayList, occurs when the CLEAR command is send by the phone
     */
    private void clear() {
        long currTime = System.currentTimeMillis();
        //calculate the elapsed time that occurred this update
        elapsedMillisThisUpdate = currTime - lastClearTime;
        //remember the current time
        lastClearTime = currTime;


        try {
            Main.drawSemaphore.acquire();
            Main.displayPoints.clear();
            Main.displayLines.clear();

            Main.displayPoints.addAll(debugPoints);
            Main.displayLines.addAll(debugLines);

            debugPoints.clear();
            debugLines.clear();

        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Main.drawSemaphore.release();
    }


}
