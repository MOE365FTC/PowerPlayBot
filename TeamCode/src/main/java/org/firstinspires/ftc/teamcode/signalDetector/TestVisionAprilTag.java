package org.firstinspires.ftc.teamcode.signalDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
public class TestVisionAprilTag extends LinearOpMode {


    @Override
    public void runOpMode()
    {
        Vision vision = new Vision(hardwareMap); //call this in MOEbot auton constructor

        //do roadrunner trajectory setup here

        while (!isStarted() && !isStopRequested()){ //init loop
            vision.visionLoop();
            telemetry.addData("Detection: ", vision.getSignalPos());
            telemetry.addData("Detections", vision.detections);
            telemetry.update();
            sleep(20);
        }

        switch(vision.getSignalPos()) {
            case 1:
                telemetry.addData("Outcome: ", "found tag 1, ran 1st path");
                break;
            case 2:
                telemetry.addData("Outcome: ", "found tag 2, ran 2nd path");
                break;
            case 3:
                telemetry.addData("Outcome: ", "found tag 3, ran 3rd path");
                break;
        }

        telemetry.update();


    }

}
