package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoPos extends OpMode {
    boolean RBflag;
    boolean LBflag;
    Servo servo;
    double pos;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        servo.setPosition(pos);
        boolean currentRB = gamepad1.right_bumper;
        boolean currentLB = gamepad1.left_bumper;
        if (currentRB && !RBflag && pos < 1.0){
            pos = pos + 0.01;
        }
        if (currentLB && !LBflag  && pos > 0.0){
            pos = pos - 0.01;
        }
        if (gamepad1.a){pos = 0.0;}
        if (gamepad1.b){pos = 0.5;}
        if (gamepad1.y){pos = 1.0;}
        RBflag = currentRB;
        LBflag = currentLB;

        telemetry.addData("POS: ", pos);
    }
}
