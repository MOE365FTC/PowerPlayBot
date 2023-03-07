package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.signalDetector.Vision;

@Disabled
@Autonomous
public class StraferAuton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int a = 29;
        int position = -1;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Vision vision = new Vision(hardwareMap); //call this in MOEbot auton constructor

        //do roadrunner trajectory setup here

        while (!isStarted() && !isStopRequested()){ //init loop
            vision.visionLoop();
            telemetry.addData("Detection: ", vision.getSignalPos());
//            telemetry.addData("Detections", vision.detections);
            telemetry.update();
            sleep(20);
            position = vision.getSignalPos();
        }

        Pose2d startPose = new Pose2d(60, 36, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .forward(52)
                .strafeRight(a)
                .strafeLeft(a)
                .strafeRight(a)
                .strafeLeft(a)
                .strafeRight(a)
                .strafeLeft(a)
                .strafeRight(a)
                .strafeLeft(a)
                .strafeRight(a)
                .strafeLeft(a)
                .build();
        TrajectorySequence case1 = drive.trajectorySequenceBuilder(traj.end())
                .strafeLeft(a)
                .build();
        TrajectorySequence case3 = drive.trajectorySequenceBuilder(traj.end())
                .strafeRight(a)
                .build();





        waitForStart();

        drive.followTrajectorySequence(traj);
        switch(position){
            case 1:
                drive.followTrajectorySequence(case1);
                break;
            case 3:
                drive.followTrajectorySequence(case3);
            default:
                break;
        }
    }
}
