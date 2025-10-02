package org.firstinspires.ftc.teamcode.pedroPathing.OpMaster;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Mecanismos {
//Tl:       INTAKE
    public CRServo intake_1;
    public CRServo intake_2;
//TL:       CANNON
    public CRServo cannon_1;
    public CRServo cannon_2;
    public CRServo cannon_3;
//Tl:       BARRIL
    public Servo barril;
//Tl:       COSOS CHISTOSOS
    public double slowModeMultiplier = 0.3; //Modo slow
//note    AprilTag search
    final double DESIRED_DISTANCE =  40;
    final double SPEED_GAIN  =  0.02  ;
    final double TURN_GAIN   =  0.01  ;
    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    public int DESIRED_TAG_ID = 0;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public AprilTagDetection desiredTag = null;

    public boolean targetFound     = false;
    public double  drive           = 0;
    public double  turn            = 0;

    public void initAll(HardwareMap hwMap){
//        intake_1 = hwMap.get(CRServo.class, "IntakeR");
//        intake_2 = hwMap.get(CRServo.class, "IntakeL");
//        cannon_1 = hwMap.get(CRServo.class, "CannonR");
//        cannon_2 = hwMap.get(CRServo.class, "CannonL");
//        cannon_3 = hwMap.get(CRServo.class, "CannonA");
//        barril = hwMap.get(Servo.class, "Barril");

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}
