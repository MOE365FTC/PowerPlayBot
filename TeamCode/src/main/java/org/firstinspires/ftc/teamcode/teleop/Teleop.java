package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class Teleop extends OpMode {
    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void init_loop(){
        telemetry.addData("headingIMU" ,robot.imu.getHeadingFirstAngle());
    }

    @Override
    public void loop() {
        robot.lift.actuate();
        robot.lift.lowerLift();
        robot.claw.actuate();
        robot.chassis.fieldCentricDrive();
//        robot.chassis.drive();
        robot.lift.forkActuate();
        if(gamepad1.a){
            robot.claw.grab();
        }
        robot.chassis.odoTelemetry(telemetry);
        telemetry.addData("lift", robot.lift.getLiftTicks());
        telemetry.update();
    }
}