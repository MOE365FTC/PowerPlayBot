package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous
public class RightAutonRR extends LinearOpMode { //test for auton using rr and markers instead of state-machine

    int strafeDistance = 20;
    int y = 8;
    int x1 = -36;
    int x2 = -55;
    int signalPos = -1; //default

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1, gamepad2); //angle - robot degree + (angle/2)

        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(x1,y))
//                .addDisplacementMarker(() -> {
//                    robot.turret.turnToDegree(-45, telemetry);
//                    robot.claw.release();
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH_GRAB);
//                            telemetry.update();
//                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(x2,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
//                    wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
                .lineToConstantHeading(new Vector2d(x1,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH_GRAB);
//                    telemetry.update();
//                })
                .lineToConstantHeading(new Vector2d(x2,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
                //wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
                .lineToConstantHeading(new Vector2d(x1,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH_GRAB);
//                    telemetry.update();
//                })
                .lineToConstantHeading(new Vector2d(x2,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
                //wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
                .lineToConstantHeading(new Vector2d(x1,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.LOW_GRAB);
//                    telemetry.update();
//                })
//////                .lineToConstantHeading(new Vector2d(x2,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
                //wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
//////                .lineToConstantHeading(new Vector2d(x1,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.LOW_GRAB);
//                    telemetry.update();
//                })
//////                .lineToConstantHeading(new Vector2d(x2,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
                //wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
////////                .lineToConstantHeading(new Vector2d(x1,y))
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.LOW_GRAB);
//                    telemetry.update();
//                })
                .build();
        TrajectorySequence case1 = drive.trajectorySequenceBuilder(traj.end())
                .lineToConstantHeading(new Vector2d(-16,y))
//                .addDisplacementMarker(() -> {
//                    robot.turret.turnToDegree(0, telemetry);
//                    telemetry.update();
//                })
                .build();
        TrajectorySequence case3 = drive.trajectorySequenceBuilder(traj.end())
                .lineToConstantHeading(new Vector2d(x2,y))
//                .addDisplacementMarker(() -> {
//                    robot.turret.turnToDegree(0, telemetry);
//                    telemetry.update();
//                })
                .build();

        while (!isStarted() && !isStopRequested()){ //init loop
            if(signalPos != -1) telemetry.addLine("Ready to start!");
            robot.vision.visionLoop();
            telemetry.addData("Detection: ", robot.vision.getSignalPos());
            sleep(20);
            signalPos = robot.vision.getSignalPos();
            telemetry.update();
        }

        waitForStart();
//        robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
        drive.followTrajectorySequence(traj);
        telemetry.addLine(String.valueOf(signalPos));
        telemetry.update();
        switch(signalPos){
            case 1:
                telemetry.addLine("Case 1");
                telemetry.update();
                drive.followTrajectorySequence(case1);
                break;
            case 3:
                telemetry.addLine("Case 3");
                telemetry.update();
                drive.followTrajectorySequence(case3);
                break;
            default:
                telemetry.addLine("Case 2");
                telemetry.update();
                break;
        }
        while(opModeIsActive()){
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("l", drive.rightRear.getCurrentPosition());
            telemetry.addData("r", drive.leftFront.getCurrentPosition());
            telemetry.addData("s", drive.leftRear.getCurrentPosition());
            telemetry.update();
        }
    }
}