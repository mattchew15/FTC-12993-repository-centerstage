package org.firstinspires.ftc.teamcode.opmode.test;

// Old imports, some not needed
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.mathCaching;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.math.MathResult;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

@Disabled
@Config
@TeleOp(name = "MathTest")
public class MathTest extends LinearOpMode {


    //DcMotorEx IntakeSlideMotor;
    DriveBase driveBase = new DriveBase(telemetry);
    LoopTime loopTime = new LoopTime(); // might have to create new instance of class instead of just declaring it (who knows)

    // uses the ElapsedTime class from the SDK to create variable GlobalTimer
    ElapsedTime GlobalTimer;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    // random setup function that runs once start is pressed but before main loop
    // this setup should be put somewhere else

    double input;
    MathResult result = new MathResult(0, 0);
    boolean prevG = false;

    @Override
    public void runOpMode()
    {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        while(opModeIsActive() && !isStopRequested())
        {
            if (gamepad1.a && !prevG)
            {
                input += 5;
                prevG= true;
            }
            if(!gamepad1.a)
            {
                prevG = false;
            }
            result = mathCaching(x -> Math.cos(Math.toRadians(x)), input, result.getPrevInput(), result.getResult());

            telemetry.addData("result", result.getResult());
            telemetry.addData("input", result.getPrevInput());
            loopTime.updateLoopTime(telemetry);
            telemetry.update();
        }
    }
}


