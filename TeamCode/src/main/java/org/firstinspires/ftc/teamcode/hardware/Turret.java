package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    public DcMotor turretMotor;
    IMU imu;
    Gamepad gamepad2;

    double ticksPerDegree = 8.56; //temp value
    int manualIncrement = 10; //amount of degrees manual control moves per press of trigger
    double maxManualTurn = 190*ticksPerDegree;

    double turretPower = 0.6;

    public Turret(HardwareMap hardwareMap, IMU imu, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
        this.imu = imu;
        turretMotor = hardwareMap.get(DcMotor.class, "TRM02");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//    public void actuate() {
//        if (gamepad2.dpad_up && gamepad2.dpad_right) {
//            //45 deg
//            turnToDegree(-45);
//        } else if (gamepad2.dpad_left && gamepad2.dpad_up) {
//            //315 deg
//            turnToDegree(45);
//        } else if (gamepad2.dpad_up) {
//            //0 deg
//            turnToDegree(0);
//        } else if (gamepad2.dpad_right) {
//            //90 deg
//            turnToDegree(-90);
//        } else if (gamepad2.dpad_left) {
//            //270 deg
//            turnToDegree(90);
//        }
//        //MANUAL CONTROL, triggers move either +10 or -10 degrees
////        if(gamepad2.left_trigger > 0.1 && turretMotor.getCurrentPosition() - (manualIncrement * ticksPerDegree) >= -maxManualTurn) { //deadzone
////            turnToDegree((int)(turretMotor.getCurrentPosition()/ticksPerDegree - manualIncrement));
////        } else if(gamepad2.right_trigger > 0.1 && turretMotor.getCurrentPosition() + (manualIncrement * ticksPerDegree) <= maxManualTurn) {
////            turnToDegree((int)(turretMotor.getCurrentPosition()/ticksPerDegree + manualIncrement));
////        }
//    }

    public void turnToDegree(int turnDegree) {
        double correctedDegrees = ((turnDegree - imu.getHeadingFirstAngle()) % 360) * ticksPerDegree; //field-centric angle
        turretMotor.setTargetPosition((int) correctedDegrees);
        turretMotor.setPower(turretPower);
    }

    public void turnToDegree(int turnDegree, Telemetry telemetry) {
        double correctedDegrees = ((turnDegree - imu.getHeadingFirstAngle()) % 360) * ticksPerDegree; //field-centric angle
        turretMotor.setTargetPosition((int) correctedDegrees);
        turretMotor.setPower(turretPower);
        telemetry.addData("Now turned to: ", (int) (correctedDegrees / ticksPerDegree) + " degrees");
    }

    public void autonCorrectedTurret(double targetX, double targetY, double currentX, double currentY){
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition() + (int) Math.toDegrees(Math.atan2((targetY-currentY),(targetX-currentX)) * ticksPerDegree));
    }

    public int getTurretMotorTicks() {
        return turretMotor.getCurrentPosition();
    }
    public int getTurretMotorTarget() {
        return turretMotor.getTargetPosition();
    }

    public void startAuton(){
        turretMotor.setTargetPosition(300);
        turretMotor.setPower(0.4);
    }

    public void straightAuton(){
        turretMotor.setTargetPosition(0);
        turretMotor.setPower(0.4);
    }

    public void startTeleOp(){
        turretMotor.setTargetPosition(0);
        turretMotor.setPower(0.4);
    }

    public void manualTurret(){
//        if(gamepad2.dpad_right){
//            turretMotor.setPower(-0.4);
//        } else if(gamepad2.dpad_left){
//            turretMotor.setPower(0.4);
//        } else{
//            turretMotor.setPower(0.0);
//        } *
        turretMotor.setPower(gamepad2.right_stick_x * 0.5);
    }
}