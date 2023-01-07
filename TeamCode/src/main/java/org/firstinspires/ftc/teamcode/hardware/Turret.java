package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    DcMotor turretMotor;
    IMU imu;
    Gamepad gamepad2;

    int ticksPerDegree = 100; //temp value
    int manualIncrement = 10; //amount of degrees manual control moves per press of trigger
    int maxManualTurn = 190*ticksPerDegree;

    double turretPower = 0.6;

    public Turret(HardwareMap hardwareMap, IMU imu, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
        this.imu = imu;
//        turretMotor = hardwareMap.get(DcMotor.class, "TRM");
//        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuate() {
        if (gamepad2.dpad_up && gamepad2.dpad_right) {
            //45 deg
            turnToDegree(45);
        } else if (gamepad2.dpad_right && gamepad2.dpad_down) {
            //135 deg
            turnToDegree(135);
        } else if (gamepad2.dpad_down && gamepad2.dpad_left) {
            //225 deg
            turnToDegree(-135);
        } else if (gamepad2.dpad_left && gamepad2.dpad_up) {
            //315 deg
            turnToDegree(-45);
        } else if (gamepad2.dpad_up) {
            //0 deg
            turnToDegree(0);
        } else if (gamepad2.dpad_right) {
            //90 deg
            turnToDegree(90);
        } else if (gamepad2.dpad_down) {
            //180 deg(
            turnToDegree(turretMotor.getCurrentPosition() > 0 ? 180 : -180);
        } else if (gamepad2.dpad_left) {
            //270 deg
            turnToDegree(-90);
        }
        //MANUAL CONTROL, triggers move either +10 or -10 degrees
        if(gamepad2.left_trigger > 0.1 && turretMotor.getCurrentPosition() - (manualIncrement * ticksPerDegree) >= -maxManualTurn) { //deadzone
            turnToDegree(turretMotor.getCurrentPosition()/ticksPerDegree - manualIncrement);
        } else if(gamepad2.right_trigger > 0.1 && turretMotor.getCurrentPosition() + (manualIncrement * ticksPerDegree) <= maxManualTurn) {
            turnToDegree(turretMotor.getCurrentPosition()/ticksPerDegree + manualIncrement);
        }
    }

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
}