package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class elevador extends OpMode {
    CRServo elevaA;
    CRServo elevaB;
    CRServo elevaC;
    CRServo elevaD;

    @Override
    public void init() {
        elevaA = hardwareMap.get(CRServo.class, "elevA");
        elevaB = hardwareMap.get(CRServo.class, "elevB");
        elevaC = hardwareMap.get(CRServo.class, "elevC");
        elevaD = hardwareMap.get(CRServo.class, "elevD");
        elevaC.setDirection(CRServo.Direction.REVERSE);
        elevaD.setDirection(CRServo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.y){
            Elevador(1);
        }else {
            Elevador(0);
        }
    }

    public void Elevador(double pow){
        elevaA.setPower(pow);
        elevaB.setPower(pow);
        elevaC.setPower(pow);
        elevaD.setPower(pow);
    }
}

