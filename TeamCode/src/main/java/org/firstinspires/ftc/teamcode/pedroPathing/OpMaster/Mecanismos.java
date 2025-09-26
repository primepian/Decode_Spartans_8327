package org.firstinspires.ftc.teamcode.pedroPathing.OpMaster;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public void initAll(HardwareMap hwMap){
        intake_1 = hwMap.get(CRServo.class, "IntakeR");
        intake_2 = hwMap.get(CRServo.class, "IntakeL");
        cannon_1 = hwMap.get(CRServo.class, "CannonR");
        cannon_2 = hwMap.get(CRServo.class, "CannonL");
        cannon_3 = hwMap.get(CRServo.class, "CannonA");
        barril = hwMap.get(Servo.class, "Barril");
    }
}
