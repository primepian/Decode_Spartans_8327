package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Servo", group = "test")
public class servoPos extends OpMode {
    boolean RBflag;
    boolean LBflag;
    boolean LTflag;
    boolean RTflag;

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
        boolean currentLT = gamepad1.left_trigger > 0.1f;
        boolean currentRT = gamepad1.right_trigger > 0.1f;
        if (currentRB && !RBflag && pos < 1.0){
            pos = pos + 0.01;
        }
        if (currentLB && !LBflag  && pos > 0.0){
            pos = pos - 0.01;
        }

        if (currentLT && !LTflag && pos < 1.0){
            pos = pos + 0.001;
        }
        if (currentRT && !RTflag  && pos > 0.0){
            pos = pos - 0.001;
        }
        if (gamepad1.a){pos = 0.0;}
        if (gamepad1.b){pos = 0.5;}
        if (gamepad1.y){pos = 1.0;}
        RBflag = currentRB;
        LBflag = currentLB;
        LTflag = currentLT;
        RTflag = currentRT;



        telemetry.addData("POS: ", pos);
    }
}
