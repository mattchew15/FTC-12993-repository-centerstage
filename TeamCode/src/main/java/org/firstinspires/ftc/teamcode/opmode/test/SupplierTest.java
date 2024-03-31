package org.firstinspires.ftc.teamcode.opmode.test;

import com.outoftheboxrobotics.photoncore.PeriodicSupplier;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.system.accessory.Log;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.AsyncSupplier;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;

import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
@Photon
@TeleOp
public class SupplierTest extends LinearOpMode
{
    LoopTime loopTime = new LoopTime();
    ColorSensor colorSensor, colorSensor1;
    PeriodicSupplier<Integer> periodicSupplier;
    private DigitalChannel RightArmLimitSwitch, LeftArmLimitSwitch;
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException
    {
//        colorSensor = hardwareMap.get(ColorSensor.class, "IntakeColourSensorFront");
//        colorSensor1 = hardwareMap.get(ColorSensor.class, "IntakeColourSensorBack");
//        TimedSupplier<Integer> timedSupplier = new TimedSupplier<>(() -> colorSensor.alpha(), 200);
//        TimedSupplier<Integer> timedSupplier1 = new TimedSupplier<>(() -> colorSensor1.alpha(), 200);

        hubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule module : hubs)
//        {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            module.clearBulkCache();
//        }
        RightArmLimitSwitch = hardwareMap.get(DigitalChannel.class, "RightArmLimit");
        LeftArmLimitSwitch = hardwareMap.get(DigitalChannel.class, "LeftArmLimit");

        //TimedSupplier<Boolean> timedSupplier = new TimedSupplier<>(() -> !RightArmLimitSwitch.getState(), 200);
        //TimedSupplier<Boolean> timedSupplier1 = new TimedSupplier<>(() -> !LeftArmLimitSwitch.getState(), 200);

        //periodicSupplier = new PeriodicSupplier<>(() -> CompletableFuture.supplyAsync(() -> colorSensor.alpha()), 200);
        //periodicSupplier.update();
        /*
        AsyncSupplier<Integer> asyncSupplier = new AsyncSupplier<>(() ->
        {
            return CompletableFuture.supplyAsync(() -> colorSensor.alpha());
        } );

         */
        double t=0;

        waitForStart();
        while (opModeIsActive())
        {
//            for (LynxModule module : hubs)
//            {
//                //module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//                module.clearBulkCache();
//            }

//double time = System.currentTimeMillis();

            boolean i = !RightArmLimitSwitch.getState();
            boolean u = !LeftArmLimitSwitch.getState();
            telemetry.addData("Right", i);
            telemetry.addData("Left", u);

//            double i = timedSupplier.get(); //periodicSupplier.get();
//            double u = timedSupplier1.get();
//            telemetry.addData("Color sensor", i);
//            telemetry.addData("Color sensor1", u);
            loopTime.updateLoopTime(telemetry);
            telemetry.update();

        }

    }

}
