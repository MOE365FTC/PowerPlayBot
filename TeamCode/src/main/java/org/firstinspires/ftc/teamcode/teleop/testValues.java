package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

public class testValues extends OpMode {

    MOEBot robot;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            robot.lift.nudgeFourBar();
        } else if (gamepad1.b) {
            robot.claw.nudgeClawR(gamepad1.left_stick_y);
        } else if (gamepad1.x){
            robot.claw.nudgeClawL(gamepad1.left_stick_y);
        }
        telemetryOut();
    }

    public void telemetryOut() {
        telemetry.addData("TurretTicks: ", robot.turret.getTurretMotorTicks());
        telemetry.addData("LiftTicks: ", robot.lift.getLiftTicksL());
        telemetry.addData("LiftTicks: ", robot.lift.getLiftTicksR());
        telemetry.addData("FourBarTicks", robot.lift.getFourBarTicks());
        telemetry.addData("ClawTicksL", robot.claw.getClawTicksL());
        telemetry.addData("ClawTicksR", robot.claw.getClawTicksR());
    }
}