package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.base.AutoCommand;
import org.firstinspires.ftc.teamcode.system.base.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.base.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

@Autonomous
public class BlueAuto extends LinearOpMode {
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final OuttakeSubsystem outtake = new OuttakeSubsystem();
    private final AutoCommand command = new AutoCommand();
    @Override
    public void runOpMode() {
        SetAuto.setBlueAuto();

        hardware.init(hardwareMap);

        // Gets position to park
        BLUE_POSITION = hardware.getBluePosition();

        GlobalTimer.reset();
        command.updateTimer();

        while (!isStarted()) {
            print.telemetryTeamProp();
            telemetry.update();
            command.init();
        }

        hardware.initAprilTag(hardwareMap);

        waitForStart();
        GlobalTimer.reset();
        command.updateTimer();

        while (opModeIsActive() && !isStopRequested()) {
            print.telemetryTeamProp();
            print.telemetryAprilTag();
            telemetry.update();

            // Main state machine
            switch (command.currentState) {

            }
        }
    }
}
