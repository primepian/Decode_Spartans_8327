package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp
public class CannonTest extends LinearOpMode {
    DcMotor CannonR;
    DcMotor CannonL;
    @Override
    public void runOpMode(){
        initall(hardwareMap);

      waitForStart();
      while (opModeIsActive()){
          if (gamepad1.right_trigger > 0.0){
              shoot(gamepad1.right_trigger);
          }
          else {
              shoot(0);
          }
      }
    }
    public void initall(HardwareMap hwmap){
        CannonR = hwmap.get(DcMotor.class, "cannonR");
        CannonL = hwmap.get(DcMotor.class, "cannonL");
        CannonR.setDirection(DcMotor.Direction.REVERSE);
        CannonL.setDirection(DcMotor.Direction.FORWARD);
    }
    public void shoot(double power){
        CannonR.setPower(power);
        CannonL.setPower(power);
    }
}
