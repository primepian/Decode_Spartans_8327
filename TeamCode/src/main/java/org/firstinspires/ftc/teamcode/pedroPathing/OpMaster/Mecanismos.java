package org.firstinspires.ftc.teamcode.pedroPathing.OpMaster;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Mecanismos {
//Tl:       INTAKE
    public DcMotor barredora;
//TL:       CANNON
    public DcMotor cannonR;
    public DcMotor cannonL;
//Tl:       BARRIL
    public Servo barril;
//Tl:       COSOS CHISTOSOS
    public double slowModeMultiplier = 0.3; //Modo slow
    public boolean invertedDrive;
    public boolean RBflag;
//note    AprilTag search
    public final double DESIRED_DISTANCE =  40;
    public final double SPEED_GAIN  =  0.02  ;
    public final double TURN_GAIN   =  0.01  ;
    public final double MAX_AUTO_SPEED = 0.5;
    public final double MAX_AUTO_TURN  = 0.3;

    public int DESIRED_TAG_ID = 0;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public AprilTagDetection desiredTag = null;

    public boolean targetFound     = false;
    public double  drive           = 0;
    public double  turn            = 0;

    public void initAll(HardwareMap hwMap){
        barredora = hwMap.get(DcMotor.class, "barredora");
        cannonR = hwMap.get(DcMotor.class, "CannonR");
        cannonL = hwMap.get(DcMotor.class, "CannonL");
        barredora.setDirection(DcMotorSimple.Direction.REVERSE);
        cannonL.setDirection(DcMotorSimple.Direction.REVERSE);
//        barril = hwMap.get(Servo.class, "Barril");

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    public void shoot(double power){
        cannonR.setPower(power);
        cannonL.setPower(power);
    }
    public void intake(double power){
        barredora.setPower(power);
    }
}
