package org.firstinspires.ftc.teamcode.signalDetector;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    //fx and fy are the focal lengths and cx, cy represent the camera principal point.
    //https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
    double fx = 578.272; //578.272 <- provided examples for a webcam, we have to tune to ours
    double fy = 578.272; //578.272
    double cx = 402.145; //402.145
    double cy = 221.506; //221.506

    // UNITS ARE METERS
    double tagsize = 0.166; //<- may have to change depending on tag we are using

    int POS_1_TAG_ID = 1; // Tag ID 1 from the 36h11 family
    int POS_2_TAG_ID = 2; // Tag ID 2 from the 36h11 family
    int POS_3_TAG_ID = 3; // Tag ID 3 from the 36h11 family

    int signalPosition;

    //ArrayList<AprilTagDetection> currentDetections;

    public Vision(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "signalCam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened(){
                camera.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT); //change depending on this year's webcam
            }

            @Override
            public void onError(int errorCode){

            }
        });
    }
    int detections = 0;
    public void visionLoop() {
        //currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        detections = currentDetections.size();
        if(currentDetections.size() != 0){

            for(AprilTagDetection tag : currentDetections){
                if(tag.id == POS_1_TAG_ID){
                    signalPosition = 1;
                    break;
                } else if(tag.id == POS_2_TAG_ID){
                    signalPosition = 2;
                    break;
                } else if(tag.id == POS_3_TAG_ID){
                    signalPosition = 3;
                    break;
                }
            }
        } else signalPosition = -1;
    }

    public int getSignalPos(){

        return signalPosition; }
}