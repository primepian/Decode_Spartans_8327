package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
@Disabled
public class MotorDirections extends OpMode {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;

    int pos = 0;
    int pos1 = 0;
    int pos2 = 0;
    int pos3 = 0;


    @Override
    public void init() {
        initmotores(hardwareMap);
        telemetry.addLine("Right trigger to move all motors." +
                "> Use Dpad to move individual motors." +
                "> Use Buttons to change motor directions");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger >= 0.0) {
            leftFront.setPower(gamepad1.right_trigger);
            rightFront.setPower(gamepad1.right_trigger);
            leftRear.setPower(gamepad1.right_trigger);
            rightRear.setPower(gamepad1.right_trigger);
        }

        if (gamepad1.a && pos == 0){
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            pos = 1;
        }else if (gamepad1.b && pos1 == 0){
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            pos1 = 1;
        }else if (gamepad1.y && pos2 == 0){
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            pos2 = 1;
        } else if (gamepad1.x && pos3 == 0) {
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            pos3 = 1;
        }
        else if (gamepad1.a && pos == 1){
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            pos = 0;
        }else if (gamepad1.b && pos1 == 1){
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            pos1 = 0;
        }else if (gamepad1.y && pos2 == 1){
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            pos2 = 0;
        } else if (gamepad1.x && pos3 == 1) {
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            pos3 = 0;
        }

        if (gamepad1.dpad_down){
            leftFront.setPower(1);
        } else if (gamepad1.dpad_right) {
            rightFront.setPower(1);
        } else if (gamepad1.dpad_up){
            leftRear.setPower(1);
        } else if (gamepad1.dpad_left) {
            rightRear.setPower(1);
        } else {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }
        telemetry.addData("[↓][A] LeftFront: ", leftFront.getDirection());
        telemetry.addData("[→][B] RightFront: ", rightFront.getDirection());
        telemetry.addData("[↑][Y] LeftRear: ", leftRear.getDirection());
        telemetry.addData("[←][X] RightRear: ", rightRear.getDirection());
        telemetry.update();
    }
    public void initmotores(HardwareMap hwMap){
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
    }

}
