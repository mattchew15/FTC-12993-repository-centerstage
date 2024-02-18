package org.firstinspires.ftc.teamcode.opmode.auto;

import static com.outoftheboxrobotics.photoncore.PhotonCore.photon;

import com.outoftheboxrobotics.photoncore.PeriodicSupplier;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;

// Ideally for testing odo and/or photon

@Photon
@TeleOp(name = "photon test")
public class AccuracyTest extends LinearOpMode
{
    //PhotonDcMotor motor1, motor2;
    LoopTime loopTime = new LoopTime();

    @Override
    public void runOpMode() throws InterruptedException
    {

        PhotonDcMotor motor1 = (PhotonDcMotor) hardwareMap.dcMotor.get("BL");
        PhotonDcMotor motor2 = (PhotonDcMotor) hardwareMap.dcMotor.get("PitchMotor");

        //When the FTC SDK display the voltage on the Driver Station, it uses the same interfaces we use. These reads happen at intervals of ~3s and cannot be disabled, so PhotonCore gives you access to this normally inaccessible data.
        PhotonLynxVoltageSensor sensor = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
        PeriodicSupplier<Double> current2 = new PeriodicSupplier<>(() -> motor2.getCurrentAsync(CurrentUnit.AMPS), 100);
        PeriodicSupplier<Double> current1 = new PeriodicSupplier<>(() -> motor2.getCurrentAsync(CurrentUnit.AMPS), 100);
        waitForStart();
        while(opModeIsActive())
        {
            double voltage = sensor.getCachedVoltage();
            telemetry.addData("Voltage", voltage);


            telemetry.addData("MaxParallelCommands", photon.maximumParallelCommands());
            telemetry.addData("SingleThreadOptimized", photon.singleThreadOptimized());

            motor1.setPower(Math.sin(System.currentTimeMillis()/1000.0));
            motor2.setPower(Math.sin(System.currentTimeMillis()/1000.0));

            //motor1.getCorrectedCurrent(CurrentUnit.AMPS);
            //motor2.getCorrectedCurrent(CurrentUnit.AMPS);



            telemetry.addData("Current1", current1.get());
            telemetry.addData("Power1", Math.sin(System.currentTimeMillis()/1000.0));

            telemetry.addData("Current2", current2.get());
            telemetry.addData("Power2", Math.sin(System.currentTimeMillis()/1000.0));

            loopTime.updateLoopTime(telemetry);
            telemetry.update();
    }
}
}
