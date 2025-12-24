package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class CannonTest extends LinearOpMode {
    DcMotor CannonR;
    DcMotor CannonL;
    Servo pateador;
    double pow = 0;
    boolean RBflag = false;
    boolean LBflag = false;
    @Override
    public void runOpMode(){
        initall(hardwareMap);
        pateador.setPosition(0.52);

      waitForStart();
      while (opModeIsActive()){
          if (gamepad1.left_trigger > 0.0){
              shoot(gamepad1.left_trigger);
          }
          if (gamepad1.right_trigger > 0.0){
              shoot(pow);
          }

          boolean currentRB = gamepad1.right_bumper;
          if (currentRB && !RBflag && pow < 1.0){
              pow = pow + 0.01;
          }
          RBflag = currentRB;

          boolean currentLB = gamepad1.left_bumper;
          if (currentLB && !LBflag && pow > 0.0){
              pow = pow - 0.01;
          }
          LBflag = currentLB;

          if (gamepad1.x){
              pow = 0.25;
              shoot(pow);
          }
          if (gamepad1.y){
              pow = 0.5;
              shoot(pow);
              
          }
          if (gamepad1.b){
              pow = 0.75;
              shoot(pow);
          }
          if (gamepad1.a){
              pow = 1.0;
              shoot(pow);
          }else {
              shoot(0);
          }
          if (gamepad1.dpad_up){
              pateador.setPosition(0.48);
          }
          if (gamepad1.dpad_down){
              pateador.setPosition(0.52);
          }

          telemetry.addData("POW: ", pow);
          telemetry.update();
      }
    }
    public void initall(HardwareMap hwmap){
        CannonR = hwmap.get(DcMotor.class, "CannonR");
        CannonL = hwmap.get(DcMotor.class, "CannonL");
        pateador = hwmap.get(Servo.class, "pateador");
        CannonR.setDirection(DcMotor.Direction.FORWARD);
        CannonL.setDirection(DcMotor.Direction.REVERSE);
    }
    public void shoot(double power){
        CannonR.setPower(power);
        CannonL.setPower(power);
        telemetry.addData("power: ", power);
        telemetry.update();
    }
}
