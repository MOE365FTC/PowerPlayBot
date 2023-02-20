package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo leftClawServo, rightClawServo;
    Gamepad gamepad1;
    Gamepad gamepad2;
    HardwareMap hardwareMap;

    double openL = 0.9, closedL = 0.5, openR = 0.2, closedR = 0.56;
    //teleop
    public Claw(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        rightClawServo = hardwareMap.get(Servo.class, "RCS01");
        leftClawServo = hardwareMap.get(Servo.class, "LCS02");
    }

    public void actuate(){
        if(gamepad1.left_bumper || gamepad2.left_bumper){
            leftClawServo.setPosition(closedL);
            rightClawServo.setPosition(closedR);
        } else if (gamepad1.right_bumper || gamepad2.right_bumper){
            leftClawServo.setPosition(openL);
            rightClawServo.setPosition(openR);
        }
    }

    public void grab(){
        leftClawServo.setPosition(closedL);
        rightClawServo.setPosition(closedR);
    }

    public void release(){
        leftClawServo.setPosition(openL);
        rightClawServo.setPosition(openR);
    }

    public double getClawTicksL() {
        return leftClawServo.getPosition();
    }

    public double getClawTicksR() {
        return rightClawServo.getPosition();
    }

    public void nudgeClawL(double stickY) {
        leftClawServo.setPosition(getClawTicksL() + 0.1 * -Math.signum(stickY));
    }

    public void nudgeClawR(double stickY) {
        rightClawServo.setPosition(getClawTicksL() + 0.1 * -Math.signum(stickY));
    }

    public void startClawPos(){
        rightClawServo.setPosition(openR);
        leftClawServo.setPosition(openL);
    }
}