package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class newTestAuto extends LinearOpMode { //test for auton using rr and markers instead of state-machine

    int strafeDistance = 20;
    double y = 8;
    int x1 = -36;
    int x2 = -58;
    int signalPos = -1; //default

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2); //angle - robot degree + (angle/2)

        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(x1, y))
                .lineToConstantHeading(new Vector2d(x1-14, y))
                .build();
        waitForStart();
        drive.followTrajectorySequence(traj);
        telemetry.update();

        while(opModeIsActive()){
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}