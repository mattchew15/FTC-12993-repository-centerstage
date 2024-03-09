package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.accessory.Log;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.TimedSupplier2;
//import org.firstinspires.ftc.teamcode.system.accessory.TimedSupplier3;

@Autonomous(name="Bulking")
public class BulkingTest extends LinearOpMode
{
    double sensor1val, sensor2val;
    ColorSensor sensor1, sensor2;
    DcMotor motor0, motor1, motor2, motor3;
    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        sensor1 = hardwareMap.get(ColorSensor.class, "1");
        sensor2 = hardwareMap.get(ColorSensor.class, "2");

        motor0 = hardwareMap.get(DcMotor.class, "0m");
        motor1 = hardwareMap.get(DcMotor.class, "1m");
        motor2 = hardwareMap.get(DcMotor.class, "2m");
        motor3 = hardwareMap.get(DcMotor.class, "3m");


        LoopTime loopTime = new LoopTime();

        TimedSupplier2<Integer> supplier1 = new TimedSupplier2<>(()-> sensor1.alpha(), 200);
        TimedSupplier2<Integer> supplier2 = new TimedSupplier2<>(()-> sensor2.alpha(), 200);
        int i = 0;
        waitForStart();
        while (!isStopRequested() && opModeIsActive())
        {

            telemetry.addData("Sensor1", supplier1.get()); //supplier1.getCached());
            telemetry.addData("Sensor2", supplier2.get());
            telemetry.addData("Cycles", i);
            loopTime.updateLoopTime(telemetry);
            telemetry.update();
            i++;
        }
    }
}
