package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "RPM Motor Control")
public class MotorConEncoder extends LinearOpMode {

    DcMotorEx motor;

    static final double TICKS_PER_REV = 537.7;
    double targetRPM = 0;
    double power = 0;

    @Override
    public void runOpMode() {


        motor = hardwareMap.get(DcMotorEx.class, "motor");

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) power = 100;
            if (gamepad1.b) power = 75;
            if (gamepad1.x) power = 50;
            if (gamepad1.y) power = 0;
            targetRPM = (power * 312) / 100;

            setMotorRPM(targetRPM);

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Power: ", power + "%");
            telemetry.addData("Current RPM", getMotorRPM());

            telemetry.update();
        }
    }

    void setMotorRPM(double rpm) {
        double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
        motor.setVelocity(ticksPerSecond);
    }

    double getMotorRPM() {
        return (motor.getVelocity() * 60.0) / TICKS_PER_REV;
    }
}
