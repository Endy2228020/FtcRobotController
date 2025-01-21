package org.firstinspires.ftc.teamcode.Camera.Childrens.WebCams;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.Childrens.PipeLines.PipeLineBluERight;
import org.firstinspires.ftc.teamcode.Camera.Childrens.PipeLines.PipelineBluELeft;
import org.firstinspires.ftc.teamcode.Camera.Parents.Webcam;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name = "CamBlueRight", group = "Auto")
public class WebcamBlueRight extends Webcam {
    @Override
    public void initCam(Telemetry telemetry){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new PipeLineBluERight(webcam, telemetry));
    }
}
