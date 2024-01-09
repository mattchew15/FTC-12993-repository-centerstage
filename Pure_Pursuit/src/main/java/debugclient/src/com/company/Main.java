package debugclient.src.com.company;


import Main.src.treamcode.MathFunctions;
import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.stage.Stage;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.concurrent.Semaphore;

import static debugclient.src.com.company.Screen.convertToScreen;



public class Main extends Application{
    //this is the ImageView that will hold the field background
    private ImageView fieldBackgroundImageView;
    private Canvas fieldCanvas;

    private Group rootGroup;//holds the grid and the field stuff

    //this will overlay stuff for other debugging purposes. This is inside the rootGroup
    private HBox mainHBox;




    //////////////////////ALL LAYOUT PARAMETERS////////////////////////
    private final int MAIN_GRID_HORIZONTAL_GAP = 100;//horizontal spacing of the main grid
    private final int MAIN_GRID_VERTICAL_GAP = 100;//vertical spacing of the main grid
    ///////////////////////////////////////////////////////////////////




    public static Semaphore drawSemaphore = new Semaphore(1);


    /**
     * Launches
     */

    public static void main(String[] args){
        launch(args);
    }

    /**
     * Runs at the initialization of the window (after main)
     */
    @Override
    public void start(Stage primaryStage) throws Exception {
        //WINDOW STUFF//
        primaryStage.setTitle("Gluten Free Debug Receiver v1.1");
        ////////////////



        //this is the group that holds everything
        rootGroup = new Group();
        //create a new scene, pass the rootGroup
        Scene scene = new Scene(rootGroup);




        //Now we can setup the HBox
        mainHBox = new HBox();
        //bind the main h box width to the primary stage width so that changes with it
        mainHBox.prefWidthProperty().bind(primaryStage.widthProperty());
        mainHBox.prefHeightProperty().bind(primaryStage.heightProperty());




        ///////////////////////////////////Setup the background image/////////////////////////////////
        Image image = new Image(Files.newInputStream(Paths.get(System.getProperty("user.dir") + "/Pure_Pursuit/src/main/java/debugclient/field.png")));
        fieldBackgroundImageView = new ImageView();

        fieldBackgroundImageView.setImage(image);//set the image

        //add the background image
        rootGroup.getChildren().add(fieldBackgroundImageView);
        //////////////////////////////////////////////////////////////////////////////////////////////




        //Setup the canvas//
        fieldCanvas = new Canvas(primaryStage.getWidth(),primaryStage.getHeight());
        //the GraphicsContext is what we use to draw on the fieldCanvas
        GraphicsContext gc = fieldCanvas.getGraphicsContext2D();
        rootGroup.getChildren().add(fieldCanvas);//add the canvas
        ////////////////////




        /**
         * We will use a vbox and set it's width to create a spacer in the window
         * USE THIS TO CHANGE THE SPACING
         */
        VBox debuggingHSpacer = new VBox();
        mainHBox.getChildren().add(debuggingHSpacer);









        Group logGroup = new Group();

        Image logImage = new Image(Files.newInputStream(Paths.get(System.getProperty("user.dir") + "/Pure_Pursuit/src/main/java/debugclient/log background.png")));
        ImageView logImageView = new ImageView();
        logImageView.setImage(logImage);//set the image

        logImageView.setFitHeight(logImage.getHeight()/2.5);
        logImageView.setFitWidth(logImage.getWidth()/2.5);

        logGroup.setTranslateY(10);
        //add the background image
        logGroup.getChildren().add(logImageView);






        Label debuggingLabel = new Label();
        debuggingLabel.setFont(new Font("Courier New",20));
        debuggingLabel.textFillProperty().setValue(new Color(0,1.0,1.0,1));
        debuggingLabel.setPrefWidth(logImageView.getFitWidth()-25);
        debuggingLabel.setLayoutX(16);
        debuggingLabel.setLayoutY(logImageView.getFitHeight()/4.7);
        debuggingLabel.setWrapText(true);


        logGroup.getChildren().add(debuggingLabel);



        mainHBox.getChildren().add(logGroup);//add the log group






        //now we can add the mainHBox to the root group
        rootGroup.getChildren().add(mainHBox);
        scene.setFill(Color.BLACK);//we'll be black
        primaryStage.setScene(scene);//set the primary stage's scene
        primaryStage.setWidth(800);
        primaryStage.setHeight(800);
        primaryStage.setMaximized(true);

        //show the primaryStage
        primaryStage.show();



        UdpUnicastClient udpUnicastClient = new UdpUnicastClient(11115);
        Thread runner = new Thread(udpUnicastClient);
        runner.start();






        //CREATE A NEW ANIMATION TIMER THAT WILL CALL THE DRAWING OF THE SCREEN
        new AnimationTimer() {
            @Override public void handle(long currentNanoTime) {
                try {
                    //acquire the drawing semaphore
                    drawSemaphore.acquire();

                    //set the width and height
                    Screen.setDimensionsPixels(scene.getWidth(),
                            scene.getHeight());
                    fieldCanvas.setWidth(Screen.getFieldSizePixels());
                    fieldCanvas.setHeight(Screen.getFieldSizePixels());

                    fieldBackgroundImageView.setFitWidth(Screen.getFieldSizePixels());
                    fieldBackgroundImageView.setFitHeight(Screen.getFieldSizePixels());

                    debuggingHSpacer.setPrefWidth(scene.getWidth() * 0.01);

                    debuggingLabel.setMaxWidth(scene.getWidth() * 0.2);


                    debuggingLabel.setText("Robot Coordinates: \n" +"X: " + MessageProcessing.getRobotX()
                    + " , Y: " + MessageProcessing.getRobotY() + "\nAngle: "
                            + String.format("%.2f", MathFunctions.AngleWrap(Math.toDegrees(MessageProcessing.getRobotAngle()))) + "°");
                    System.out.println(primaryStage.getWidth());
//                    gc.setLineWidth(10);
                    drawScreen(gc);

                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                drawSemaphore.release();

            }
        }.start();
    }


    /**
     * This will draw the screen using the graphics context
     * @param gc the graphics context
     */
    private void drawScreen(GraphicsContext gc) {
        //clear everything first
        gc.clearRect(0,0,Screen.widthScreen,Screen.heightScreen);
//        gc.fillRect(0,0,Screen.widthScreen,Screen.heightScreen);
        //then draw the robot
        drawRobot(gc);
        //draw all the lines and points retrieved from the phone
        drawDebugLines(gc);
        drawDebugPoints(gc);

    }



    public static ArrayList<floatPoint> displayPoints = new ArrayList<>();//all the points to display
    public static ArrayList<Line> displayLines = new ArrayList<>();//all the lines to display

    private void drawDebugPoints(GraphicsContext gc) {
        for(int i =0; i < displayPoints.size(); i ++){
            floatPoint displayLocation = convertToScreen(
                    new floatPoint(displayPoints.get(i).x, displayPoints.get(i).y));
            double radius = 5;
            gc.setStroke(new Color(0.0,1.0,1.0,0.6));

            gc.strokeOval(displayLocation.x-radius,displayLocation.y-radius,2*radius,2*radius);
        }

        for(int i =0; i < MessageProcessing.pointLog.size(); i ++){
            floatPoint displayLocation = convertToScreen(
                    new floatPoint(MessageProcessing.pointLog.get(i).x,
                            MessageProcessing.pointLog.get(i).y));
            double radius = 5;
            gc.setFill(new Color(1.0,0.0 + (double) i/MessageProcessing.pointLog.size(),0,0.9));

            gc.fillOval(displayLocation.x-radius,displayLocation.y-radius,2*radius,2*radius);

        }


    }
    private void drawDebugLines(GraphicsContext gc) {
        for(int i =0; i < displayLines.size(); i ++){
            floatPoint displayLocation1 = convertToScreen(
                    new floatPoint(displayLines.get(i).x1, displayLines.get(i).y1));
            floatPoint displayLocation2 = convertToScreen(
                    new floatPoint(displayLines.get(i).x2, displayLines.get(i).y2));


            gc.setLineWidth(3);
            gc.setStroke(new Color(0.0,1.0,1.0,0.6));


            gc.strokeLine(displayLocation1.x,displayLocation1.y,displayLocation2.x,displayLocation2.y);
        }
    }



    /**
     * This will move the background image and everything else to follow the robot
     */
    private void followRobot(double robotX, double robotY){
        //set the center point to the robot
//        Screen.setCenterPoint(robotX, robotY);
        Screen.setCenterPoint(Screen.getCentimetersPerPixel()*Screen.widthScreen/2.0,
                Screen.getCentimetersPerPixel()*Screen.heightScreen/2.0);

        //get where the origin of the field is in pixels
        floatPoint originInPixels = convertToScreen(new floatPoint(0,Screen.ACTUAL_FIELD_SIZE));
        fieldBackgroundImageView.setX(originInPixels.x);
        fieldBackgroundImageView.setY(originInPixels.y);
    }




    //the last position of the robot
    double lastRobotX = 0;
    double lastRobotY = 0;
    double lastRobotAngle = 0;

    /**
     * Draws the robot
     * @param gc the graphics context
     */
    private void drawRobot(GraphicsContext gc) {
        //robot radius is half the diagonal length
        double robotRadius = Math.sqrt(2) * 18.0 * 2.54 / 2.0;

        double robotX = MessageProcessing.getInterpolatedRobotX();
        double robotY = MessageProcessing.getInterpolatedRobotY();
        double robotAngle = MessageProcessing.getInterpolatedRobotAngle();

        followRobot(robotX,robotY);



        double topLeftX = robotX + (robotRadius * (Math.cos(robotAngle+ Math.toRadians(45))));
        double topLeftY = robotY + (robotRadius * (Math.sin(robotAngle+ Math.toRadians(45))));
        double topRightX = robotX + (robotRadius * (Math.cos(robotAngle- Math.toRadians(45))));
        double topRightY = robotY + (robotRadius * (Math.sin(robotAngle- Math.toRadians(45))));
        double bottomLeftX = robotX + (robotRadius * (Math.cos(robotAngle+ Math.toRadians(135))));
        double bottomLeftY = robotY + (robotRadius * (Math.sin(robotAngle+ Math.toRadians(135))));
        double bottomRightX = robotX + (robotRadius * (Math.cos(robotAngle- Math.toRadians(135))));
        double bottomRightY = robotY + (robotRadius * (Math.sin(robotAngle- Math.toRadians(135))));

        Color c = Color.color(1.0,1.0,0.0);
        //draw the points
//        drawLineField(gc,topLeftX, topLeftY, topRightX, topRightY,c);
//        drawLineField(gc,topRightX, topRightY, bottomRightX, bottomRightY,c);
//        drawLineField(gc,bottomRightX, bottomRightY, bottomLeftX, bottomLeftY,c);
//        drawLineField(gc,bottomLeftX, bottomLeftY, topLeftX, topLeftY,c);
//

        try {
            floatPoint bottomLeft = convertToScreen(new floatPoint(topLeftX,topLeftY));
            double width = 1.0/Screen.getCentimetersPerPixel() * 18*2.54;//calculate the width of the image in pixels

            gc.save();//save the gc
            gc.transform(new Affine(new Rotate(Math.toDegrees(-robotAngle) + 90, bottomLeft.x, bottomLeft.y)));
            Image image = new Image(new FileInputStream(System.getProperty("user.dir") + "/Pure_Pursuit/src/main/java/debugclient/robot.png"));
            gc.drawImage(image,bottomLeft.x, bottomLeft.y,width,width);


            gc.restore();

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }





    public void drawLineField(GraphicsContext gc,double x1, double y1, double x2, double y2,Color color){
        floatPoint first = convertToScreen(new floatPoint(x1,y1));
        floatPoint second = convertToScreen(new floatPoint(x2,y2));
        gc.setStroke(color);
        gc.strokeLine(first.x,first.y,second.x,second.y);
        gc.setStroke(Color.BLACK);
    }


}