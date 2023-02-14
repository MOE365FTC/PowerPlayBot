package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class Teleop extends OpMode {
    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
//        robot.lift.startFourBarPos();
//        robot.claw.startClawPos();
    }

    @Override
    public void init_loop(){
        telemetry.addData("headingIMU" ,robot.imu.getHeadingFirstAngle());
    }

    @Override
    public void loop() {
        robot.lift.actuate();
//        robot.claw.actuate();
//        robot.turret.actuate();
//        robot.chassis.fieldCentricDrive();
//        telemetry.addData("turret target", robot.turret.getTurretMotorTarget());
//        telemetry.addData("turret pos", robot.turret.getTurretMotorTicks());
//        telemetry.addData("heading imu" ,robot.imu.getHeadingFirstAngle());
        telemetry.addData("left lift", robot.lift.getLiftTicksL());
        telemetry.addData("right lift", robot.lift.getLiftTicksR());
    }
}
