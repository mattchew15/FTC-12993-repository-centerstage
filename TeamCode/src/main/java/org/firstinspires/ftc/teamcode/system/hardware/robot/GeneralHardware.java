package org.firstinspires.ftc.teamcode.system.hardware.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;

/*import org.firstinspires.ftc.teamcode.Modules.DriveModules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.DriveModules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Wrappers.Analog30cm;
import org.firstinspires.ftc.teamcode.Wrappers.CoolDigitalSensor;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;*/

public class GeneralHardware {
    public HardwareMap hardwareMap;

    public DcMotorEx mch0, mch1, mch2, mch3;
    public DcMotorEx meh0, meh1, meh2, meh3;

    public Encoder ech0, ech1, ech2, ech3;
    public Encoder eeh0, eeh1, eeh2, eeh3;

    public Servo sch0, sch1, sch2, sch3, sch4, sch5;
    public Servo seh0, seh1, seh2, seh3, seh4, seh5;

    public ImuThread imu;

    public Localizer localizer;

    public VoltageSensor voltageSensor;
    public static double voltage;

    public static boolean AUTO = false;

    public DigitalChannel dc0, dc1;

    public enum Color{
        Red, Blue, Universal
    }

    public final Color color;

    public GeneralHardware(HardwareMap hm, Color color){
        this(hm, color, false);
    }

    public GeneralHardware(HardwareMap hm, Color color, boolean reset){
        this.hardwareMap = hm;
        this.color = color;

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

//        chub = hardwareMap.getAll(LynxModule.class).get(0).isParent() ?
//                hardwareMap.getAll(LynxModule.class).get(0) : hardwareMap.getAll(LynxModule.class).get(1);
//
//        ehub = hardwareMap.getAll(LynxModule.class).get(1).isParent() ?
//                hardwareMap.getAll(LynxModule.class).get(0) : hardwareMap.getAll(LynxModule.class).get(1);

        mch0 = hm.get(DcMotorEx.class, "ch0");
        mch1 = hm.get(DcMotorEx.class, "ch1");
        mch2 = hm.get(DcMotorEx.class, "ch2");
        mch3 = hm.get(DcMotorEx.class, "ch3");

        meh0 = hm.get(DcMotorEx.class, "eh0");
        meh1 = hm.get(DcMotorEx.class, "eh1");
        meh2 = hm.get(DcMotorEx.class, "eh2");
        meh3 = hm.get(DcMotorEx.class, "eh3");

        ech0 = new Encoder(mch0);
        ech1 = new Encoder(mch1);
        ech2 = new Encoder(mch2);
        ech3 = new Encoder(mch3);

        eeh0 = new Encoder(meh0);
        eeh1 = new Encoder(meh1);
        eeh2 = new Encoder(meh2);
        eeh3 = new Encoder(meh3);

        if(reset){
            mch0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mch1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mch3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            meh0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            meh1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            meh2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            meh3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            mch0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mch1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mch2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mch3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            meh0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            meh1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            meh2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            meh3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        sch0 = hm.get(Servo.class, "sch0");
        sch1 = hm.get(Servo.class, "sch1");
        sch2 = hm.get(Servo.class, "sch2");
        sch3 = hm.get(Servo.class, "sch3");
        sch4 = hm.get(Servo.class, "sch4");
        sch5 = hm.get(Servo.class, "sch5");

        seh0 = hm.get(Servo.class, "seh0");
        seh1 = hm.get(Servo.class, "seh1");
        seh2 = hm.get(Servo.class, "seh2");
        seh3 = hm.get(Servo.class, "seh3");
        seh4 = hm.get(Servo.class, "seh4");
        seh5 = hm.get(Servo.class, "seh5");

        voltageSensor = hardwareMap.voltageSensor.iterator().next(); //TODO: supplier version

        dc0 = hm.get(DigitalChannel.class, "dc0");
        dc1 =hm.get(DigitalChannel.class, "dc1");

        imu = new ImuThread(hm);

        localizer = new Localizer(this);

        MecanumDrive.headingMultiplier = 1;
    }

    public void startThreads(LinearOpMode opMode){
//        imu.startIMUThread(opMode, localizer);
    }

    public void update(){
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }
        voltage = voltageSensor.getVoltage();
        //imu.update();
        localizer.update();
    }


}