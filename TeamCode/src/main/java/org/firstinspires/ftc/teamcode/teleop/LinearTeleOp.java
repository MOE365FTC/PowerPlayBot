package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

@TeleOp
public class LinearTeleOp extends LinearOpMode {
    MOEBot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();
        Pose2d poseEstimate = drive.getPoseEstimate();
        Pose2d targetPose = poseEstimate;
        while (opModeIsActive() && !isStopRequested()) {
            robot.lift.actuate();
            robot.lift.manualLift();
            robot.claw.actuate();
//        robot.chassis.fieldCentricDrive();
            robot.chassis.drive();
            robot.lift.forkActuate();
            if (gamepad1.a) {
                robot.claw.grab();
            }

            drive.update();
            poseEstimate = drive.getPoseEstimate();

            targetPose = new Pose2d(targetPose.getX() + gamepad1.left_stick_x,
                                    targetPose.getY() - gamepad1.left_stick_y,
                                targetPose.getHeading() + Math.toRadians(gamepad1.right_stick_x * 24));
            if(targetPose.getX() != poseEstimate.getX() || targetPose.getY() != poseEstimate.getY() || targetPose.getHeading() != poseEstimate.getHeading()) {
                Trajectory traj = drive.trajectoryBuilder(poseEstimate).splineToSplineHeading(targetPose, 0).build();
                drive.followTrajectory(traj);
            }

            if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0){
                targetPose = poseEstimate;
            }
            telemetry.addData("lift", robot.lift.getLiftTicks());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("targetX", targetPose.getX());
            telemetry.addData("targetY", targetPose.getY());
            telemetry.addData("targetHeading", targetPose.getHeading());
            telemetry.update();
        }
    }
}
