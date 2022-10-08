package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.IMU;

public class RightAuton extends OpMode {
    MOEBot robot = MOEBot(hardwareMap, gamepad1, IMU);

    @Override
    public void init() {

    }

    @Override
    public void init_loop(){
        robot.vision.visionLoop();
        telemetry.addData("Detection: ", robot.vision.getSignalPos());
//            telemetry.addData("Detections", vision.detections);
        telemetry.update();
        robot.position = robot.vision.getSignalPos();
    }

    @Override
    public void loop() {

    }
}
