package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

public class testValues extends OpMode {

    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
        robot.claw.startClawPos();
    }

    @Override
    public void loop() {
        robot.lift.manualLift();
        if(gamepad1.b) {
            robot.claw.nudgeClaw(gamepad1.left_stick_y);
        } else if(gamepad1.a) {
            robot.lift.forkActuate(true); //up
        } else if(gamepad1.x) {
            robot.lift.forkActuate(false); //down
        }
        telemetryOut();
    }

    public void telemetryOut() {
        telemetry.addData("LiftTicks: ", robot.lift.getLiftTicks());
        telemetry.addData("ClawTicks: ", robot.claw.getClawTicks());
    }
}