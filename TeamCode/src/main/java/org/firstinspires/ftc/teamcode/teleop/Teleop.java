package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class Teleop extends OpMode {
    MOEBot robot;
    VoltageSensor vs;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
//        vs = hardwareMap.get(VoltageSensor.class, "VS");
        robot.claw.startClawPos();
    }

    @Override
    public void init_loop(){
        telemetry.addData("headingIMU" ,robot.imu.getHeadingFirstAngle());
    }

    @Override
    public void loop() {
        robot.lift.actuate();
        robot.lift.manualLift();
//        if(vs.getVoltage() < 10.5){
//            gamepad1.rumble(10);
//            gamepad2.rumble(10);
//        }
        robot.claw.actuate();
        robot.chassis.fieldCentricDrive();
        telemetry.addData("lift", robot.lift.getLiftTicks());
        telemetry.addData("flm", robot.chassis.frontLeftMotor.getPortNumber());
        telemetry.addData("brm", robot.chassis.backRightMotor.getPortNumber());
        telemetry.addData("frm", robot.chassis.frontRightMotor.getPortNumber());
        telemetry.addData("blm", robot.chassis.backLeftMotor.getPortNumber());
    }
}