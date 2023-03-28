package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class tempTeleop extends OpMode {
    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
        robot.claw.startClawPos();
    }

    @Override
    public void init_loop(){
        telemetry.addData("headingIMU" ,robot.imu.getHeadingFirstAngle());
    }

    @Override
    public void loop() {
        robot.lift.actuate();
        robot.claw.actuate();
        robot.chassis.fieldCentricDrive();
        robot.lift.manualLift();
        telemetry.addData("heading imu" ,robot.imu.getHeadingFirstAngle());
        telemetry.addData("lift", robot.lift.getLiftTicks());
    }
}