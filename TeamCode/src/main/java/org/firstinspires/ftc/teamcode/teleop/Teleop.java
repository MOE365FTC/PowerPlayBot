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
    }

    @Override
    public void loop() {
        robot.lift.actuate();
        robot.claw.actuate();
        robot.turret.actuate();
        robot.chassis.fieldCentricDrive();
    }
}
