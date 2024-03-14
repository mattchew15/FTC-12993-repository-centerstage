package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.LIFT_HITS_WHILE_PITCHING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.PITCH_DEFAULT_DEGREE_TICKS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.accessory.OuttakeInverseKinematics;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@TeleOp(name="Backdrop Kinematics")
public class BackdropKinTest extends LinearOpMode
{
    double outtakeLiftAdjustTimer,
            verticalHeight,
            pitchTarget,
            liftTarget,
            robotAngle;

    ElapsedTime GlobalTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {

        OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
        DriveBase driveBase = new DriveBase();
        OuttakeInverseKinematics outtakeInverseKinematics = new OuttakeInverseKinematics();

        outtakeSubsystem.initOuttake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);

        robotAngle = 0;
        verticalHeight = 10;

        waitForStart();
        if (opModeIsActive()) {
            outtakeSubsystem.hardwareSetup();
            driveBase.drivebaseSetup();

            GlobalTimer.reset();
            while (!isStopRequested() && opModeIsActive())
            {

                if (gamepad1.left_trigger > 0.2)
                {
                    driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
                else
                {
                    //in
                    verticalHeight += gamepad1.left_stick_y * 0.8;
                    pitchTarget = (int)outtakeInverseKinematics.pitchEnd(verticalHeight, robotAngle);
                    liftTarget = (int)outtakeInverseKinematics.slideEnd(verticalHeight, robotAngle);

                    //RAIL_SERVO_POSITION = (int)outtakeInverseKinematics.railEnd(something,backdropRelativeHeight,RAIL_SERVO_POSITION);
                    // idk if we need specific rail adjustment here

                }
                //outtakeSubsystem.liftTo((int) liftTarget, outtakeSubsystem.liftPosition,1);
                //outtakeSubsystem.pitchTo((int) pitchTarget, outtakeSubsystem.pitchEncoderPosition,1);
                telemetry.addData("Vertical Height", verticalHeight);
                telemetry.addData("Pitch Target", pitchTarget);
                telemetry.addData("Lift Target", liftTarget);
                telemetry.update();

            }
        }
    }
}

        /*
        public void fineAdjust(double fineAdjust, double timer) {
            if (timer - outtakeLiftAdjustTimer > 100) {
                double newValue = ticksToInchesSlidesMotor(liftTarget) + fineAdjust;
                liftTarget = (int) ticksToInchesSlidesMotor(outtakeInverseKinematics.slideEnd(newValue));
                //liftTarget = Math.min(liftTarget, LIFT_INCHES_FOR_MAX_EXTENSION);
                liftToInternalPIDTicks(liftTarget, 1);
                pitchToInternalPID((int) outtakeInverseKinematics.pitchEnd(newValue), 1);
                outtakeLiftAdjustTimer = timer;
            }
        }
        */

        /*
        public void fineAdjustLift(double fineAdjust, double timer) {
            if (timer - outtakeLiftAdjustTimer > 100){ // only set the position every 15 ms, once achieved cache the timer value
                backdropRelativeHeight += fineAdjust * 0.8;
                outtakeLiftAdjustTimer = GlobalTimer.milliseconds(); // cache the value of the outtakeLiftAdjustTimer
            }
        }


         */
