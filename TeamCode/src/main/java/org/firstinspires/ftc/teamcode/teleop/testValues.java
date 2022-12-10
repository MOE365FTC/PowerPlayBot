package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

public class testValues extends OpMode {

    MOEBot robot;

    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            robot.lift.nudgeFourBar();
        } else if (gamepad1.b) {
            robot.claw.nudgeClaw();
        }
        telemetryOut();
    }

    public void telemetryOut() {
        telemetry.addData("TurretTicks: ", robot.turret.getTurretMotorTicks());
        telemetry.addData("LiftTicks: ", robot.lift.getLiftTicks());
        telemetry.addData("FourBarTicks", robot.lift.getFourBarTicks());
        telemetry.addData("ClawTicks", robot.claw.getClawTicks());
    }
}
