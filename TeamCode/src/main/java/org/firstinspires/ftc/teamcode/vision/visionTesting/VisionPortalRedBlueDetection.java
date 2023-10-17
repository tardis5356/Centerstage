package org.firstinspires.ftc.teamcode.vision.visionTesting;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Autonomous(name = "Vision Test")
public class VisionPortalRedBlueDetection extends LinearOpMode {

    private VisionPortal portal;
    private RedPropDetection redPropThreshold;

    @Override
    public void runOpMode() throws InterruptedException {



        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(redPropThreshold)
                .build();

//            portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d"));

        waitForStart();
        telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
        telemetry.update();                        //Will output prop position on Driver Station Console




    }
}