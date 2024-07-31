package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.paths.splines.BelzierCurve;
import org.opencv.core.Point;

import java.util.ArrayList;
@Autonomous
public class CurvesTest extends LinearOpMode
{
    TelemetryPacket packet;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    BelzierCurve curve = new BelzierCurve(
            new Point[]{new Point(0, 0),
            new Point(0, 15), new Point(15, 15)
            });

    @Override
    public void runOpMode() throws InterruptedException
    {
        ArrayList<Point> curvePoints = curve.returnCurve();
        waitForStart();
        while(opModeIsActive())
        {
            packet = new TelemetryPacket();

            for (int i = 0; i == curvePoints.size(); i++)
            {
                packet.fieldOverlay().setFill("black").fillCircle(curvePoints.get(i).x, curvePoints.get(i).y, 1);
            }


            dashboard.sendTelemetryPacket(packet);
        }
    }
}
