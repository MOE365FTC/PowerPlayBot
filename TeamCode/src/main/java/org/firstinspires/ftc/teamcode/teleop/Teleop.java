package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@Config
@TeleOp
public class Teleop extends OpMode {
    MOEBot robot;
    VoltageSensor vs;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
//        vs = hardwareMap.get(VoltageSensor.class, "VS");
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
        robot.lift.manualLift();
//        if(vs.getVoltage() < 10.5){
//            gamepad1.rumble(10);
//            gamepad2.rumble(10);
//        }
        robot.claw.actuate();
//        robot.turret.actuate();
        robot.turret.manualTurret();
        robot.chassis.fieldCentricDrive();
//        telemetry.addData("turret target", robot.turret.getTurretMotorTarget());
//        telemetry.addData("turret pos", robot.turret.getTurretMotorTicks());
//        telemetry.addData("heading imu" ,robot.imu.getHeadingFirstAngle());
        telemetry.addData("left lift", robot.lift.getLiftTicksL());
        telemetry.addData("right lift", robot.lift.getLiftTicksR());
        telemetry.addData("flm", robot.chassis.frontLeftMotor.getPortNumber());
        telemetry.addData("brm", robot.chassis.backRightMotor.getPortNumber());
        telemetry.addData("frm", robot.chassis.frontRightMotor.getPortNumber());
        telemetry.addData("blm", robot.chassis.backLeftMotor.getPortNumber());
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("vel", robot.lift.liftMotorL.getVelocity());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
    }
}
