package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo leftClawServo, rightClawServo;
    Gamepad gamepad1;
    HardwareMap hardwareMap;

    double openL = 1.0, closedL = 0.0, openR = 0.0, closedR = 1.0;
    //teleop
    public Claw(HardwareMap hardwareMap, Gamepad gamepad1){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;

        leftClawServo = hardwareMap.get(Servo.class, "LCM");
        rightClawServo = hardwareMap.get(Servo.class, "RCM");
    }

    public void actuate(){
        if(gamepad1.left_bumper){
            leftClawServo.setPosition(closedL);
            rightClawServo.setPosition(closedR);
        } else if (gamepad1.right_bumper){
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
}