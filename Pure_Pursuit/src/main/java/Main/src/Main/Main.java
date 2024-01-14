package Main.src.Main;

import Main.src.RobotUtilities.MovementVars;
import Main.src.com.company.ComputerDebugging;
import Main.src.com.company.FloatPoint;
import Main.src.com.company.Robot;
import Main.src.com.company.UdpServer;
import Main.src.treamcode.MyOpMode;
import Main.src.treamcode.OpMode;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class Main {


    public static void main(String[] args) {
        new Main().run();
    }

    /**
     * The program runs here
     */
    public static void run() {
        //this is a test of the coding
        ComputerDebugging computerDebugging = new ComputerDebugging();
        Robot robot = new Robot();
        OpMode opMode = new MyOpMode();
        opMode.init();
        int i = 0;

        ComputerDebugging.clearLogPoints();


        long startTime = System.currentTimeMillis();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while (true) {
            if (i > 700)
            {
                ComputerDebugging.clearLogPoints();
                i = 0;
            }
            opMode.loop();

            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.update();
            ComputerDebugging.sendRobotLocation(robot);
            ComputerDebugging.sendLogPoint(new FloatPoint(Robot.worldXPosition, Robot.worldYPosition));
            ComputerDebugging.markEndOfUpdate();
            i++;
        }
    }


}
