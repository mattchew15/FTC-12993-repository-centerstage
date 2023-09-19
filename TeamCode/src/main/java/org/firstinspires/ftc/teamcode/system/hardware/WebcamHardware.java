package org.firstinspires.ftc.teamcode.system.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WebcamHardware { // experimenting, will be deleted later
    private final VisionHardware visionHardware = new VisionHardware();

    public void initWebcam(HardwareMap hwMap) {
        if (Globals.BLUE_AUTO || Globals.RED_AUTO) {
            visionHardware.initApriltag(hwMap);

            if (Globals.BLUE_AUTO) {
                visionHardware.initBlueWebcam(hwMap);

            } else if (Globals.RED_AUTO) {
                visionHardware.initRedWebcam(hwMap);

            }
        }
    }

    public void telemetryWebcam(Telemetry telemetry) {
        if (Globals.BLUE_AUTO || Globals.RED_AUTO) {
            visionHardware.telemetryAprilTag(telemetry);

            if (Globals.BLUE_AUTO) {
                visionHardware.telemetryBlueWebcam(telemetry);

            } else if (Globals.RED_AUTO) {
                visionHardware.telemetryRedWebcam(telemetry);

            }
        }
    }
}
