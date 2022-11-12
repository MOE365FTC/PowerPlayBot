package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    int signalPos = -1; //default

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MOEBot robot = new MOEBot(hardwareMap, gamepad1); //angle - robot degree + (angle/2)

        Pose2d startPose = new Pose2d(60, 36, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .forward(52)
//                .addDisplacementMarker(() -> {
//                    robot.turret.turnToDegree(-45, telemetry);
//                    robot.claw.release();
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH_GRAB);
//                            telemetry.update();
//                })
                .strafeRight(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
//                    wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
                .strafeLeft(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                    //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH_GRAB);
//                    telemetry.update();
//                })
                .strafeRight(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
                    //wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
                .strafeLeft(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                    //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH_GRAB);
//                    telemetry.update();
//                })
                .strafeRight(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
                    //wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
                .strafeLeft(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                    //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.LOW_GRAB);
//                    telemetry.update();
//                })
                .strafeRight(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
                    //wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
                .strafeLeft(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                    //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.LOW_GRAB);
//                    telemetry.update();
//                })
                .strafeRight(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.grab();
//                    robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
                    //wait till cone is securely grabbed
//                    robot.turret.turnToDegree(-45, telemetry);
//                    telemetry.update();
//                })
                .strafeLeft(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.claw.release();
                    //retract lift
//                    robot.turret.turnToDegree(90, telemetry);
//                    robot.lift.autonActuate(Lift.autonLiftPos.LOW_GRAB);
//                    telemetry.update();
//                })
                .build();
        TrajectorySequence case1 = drive.trajectorySequenceBuilder(traj.end())
                .strafeLeft(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.turret.turnToDegree(0, telemetry);
//                    telemetry.update();
//                })
                .build();
        TrajectorySequence case3 = drive.trajectorySequenceBuilder(traj.end())
                .strafeRight(strafeDistance)
//                .addDisplacementMarker(() -> {
//                    robot.turret.turnToDegree(0, telemetry);
//                    telemetry.update();
//                })
                .build();

        while (!isStarted() && !isStopRequested()){ //init loop
//            robot.vision.visionLoop();
//            telemetry.addData("Detection: ", robot.vision.getSignalPos());
//            telemetry.addData("Detections", vision.detections);
//            telemetry.update();
//            sleep(20);
//            signalPos = robot.vision.getSignalPos();
        }

        waitForStart();
//        robot.lift.autonActuate(Lift.autonLiftPos.HIGH);
        drive.followTrajectorySequence(traj);
        switch(signalPos){
            case 1:
                drive.followTrajectorySequenceAsync(case1);
                break;
            case 3:
                drive.followTrajectorySequenceAsync(case3);
                break;
            default:
                break;
        }
    }
}
