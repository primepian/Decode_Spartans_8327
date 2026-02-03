package org.firstinspires.ftc.teamcode.pedroPathing.OpMaster;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Tests.TestColorSensorMecanism;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 *>>Control Hub:
 * motores:
 *  ("rightFront")  //0
 *  ("rightRear")    //2
 *  ("leftRear")  //3
 *  ("leftFront")    //1
 * Servos:
 *   barril = 0
 *   PATEADOR = 1
 *   intakeR = x
 *   intakeL = X
 * I2C:
 *  colorSensor = 1
 *  pinpoint = 2

 *>>Expansion Hub;
 * motores:
 *  CannonL = 0
 *  Intake = 1
 *  CannonR = 2
 * servos:
 *  pateador = 0
 */

public class Mecanismos {
// Tl:========= INTAKE =========
    public DcMotor intake;
// TL:======== CANNON ==========
    public DcMotorEx cannonR;
    public DcMotorEx cannonL;
    public DcMotorEx cannon;
    public DcMotor turret;
    public Servo piringola;
// Tl:======== BARRIL ==========
    public Servo barril;
    public Servo pateador;
    NormalizedColorSensor colorSensor;
    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN,
    }
// Tl: ======== Posiciones =========
    public static final double  Ain = 0.6;
    public static final double  Bin = 0.516;
    public static final double  Cin = 0.433;

    public static final double  Aout = 0.48;
    public static final double  Bout = 0.4;
    public static final double  Cout = 0.33;
    public static final double  pateador_off = 0.55;
    public static final double  pateador_on = 0.6;

    char actualPos = 'a';

    public static double pow1 = 50;
    static final double TICKS_PER_REV = 537.7;

    //NOTE: 0 = empty || 1 = PURPLE |
    public int A = 0;
    public int B = 0;
    public int C = 0;

    //TL: ======= MODES =========
    public boolean PPG = false;
    public boolean PGP = false;
    public boolean GPP = false;

    //TL: ======= SHOOTING =========
    public boolean isShooting = false;
    public int shootStep = 0;
    public long shootStartTime = 0;

    //TL: ======= TIMES =========
    public final long PATEADOR_ON_TIME = 800;
    public final long PATEADOR_OFF_TIME = 1200;
    public final long OUTTAKE_HOLD_TIME_MS = 1800;
    public long lastIntakeTime = 0;
    public long INTAKE_COOLDOWN_MS = 500;
    public long NO_INTAKE_COOLDOWN_MS = 500;

    //Tl: ======== VARIABLES =========
    public double slowModeMultiplier = 0.3; //Modo slow
    public int checkStep = 0;
    public boolean invertedDrive;
    public boolean RBflag;
    public boolean RB2flag;
    public boolean Backflag;

    public boolean bflag;
    public boolean cflag;
    public boolean Back;
    public boolean check;

    //Tl:    AprilTag search.
    public final double DESIRED_DISTANCE = 114;
    public final double SPEED_GAIN  =  0.02; //0.02
    public final double TURN_GAIN   =  0.02; //0.01
    public final double MAX_AUTO_SPEED = 1;
    public final double MAX_AUTO_TURN  = 1;

    public int DESIRED_TAG_ID = 0;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public AprilTagDetection desiredTag = null;

    public boolean targetFound     = false;
    public double  drive           = 0;
    public double  turn            = 0;

//TL: ============= INIT =================
    public void initAll(HardwareMap hwMap){
        barril = hwMap.get(Servo.class,"servo");
//note: ---- intake
        pateador = hwMap.get(Servo.class, "pateador");
        intake = hwMap.get(DcMotor.class, "Intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//note: ---- cannon
        cannonR = hwMap.get(DcMotorEx.class, "CannonR");
        cannonL = hwMap.get(DcMotorEx.class, "CannonL");
        cannon = hwMap.get(DcMotorEx.class, "Cannon");
        cannonL.setDirection(DcMotorEx.Direction.REVERSE);
        cannon.setDirection(DcMotorEx.Direction.REVERSE);
        piringola = hwMap.get(Servo.class, "piringola");

        cannonR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        cannonR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        cannonR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        cannonL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        cannonL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        cannonL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//note: ---- sensor
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(10);

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
//note: ---- Pos
//        intakerOFF();
    }

//TL: ============== INTAKE ==============
    public void intake(double pow){
        intake.setPower(pow);
    }
    public void piringolaOFF(){
        piringola.setPosition(0.5);
    }
    public void piringolaON(){
        piringola.setPosition(0);
    }

//TL: ============ CANNON ===============
    public void shootPow(double power){
        double rpm = (312 * power) / 100;
        double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
        cannonR.setVelocity(ticksPerSecond);
        cannonL.setVelocity(ticksPerSecond);
    }
    public double getPow(){
        return (((cannonR.getVelocity() * 60) / TICKS_PER_REV) * 100 ) / 312;}
    public void shoot(){
        if (!isShooting) {
            shootPow(pow1);
            cannon.setPower(0.5);
            piringolaOFF();
            isShooting = true;
            shootStep = 0;
            shootStartTime = System.currentTimeMillis();
        }}
    public void shootNear(){
        if (!isShooting) {
            shootPow(pow1);
            cannon.setPower(0.5);
            piringolaON();
            isShooting = true;
            shootStep = 0;
            shootStartTime = System.currentTimeMillis();
        }
    }



//TL: ============= BARRREL =============
    public void G28(){
        if (A == 0 && B == 0 && C == 0) {
            barril.setPosition(Ain);
            actualPos = 'a';
            isShooting = false;
        }
        if (A != 0 && B != 0 && C != 0){
            barril.setPosition(0.598);
            actualPos = 'a';
        }
    }
    //note: MAIN
    public void shootingandIntake(Telemetry telemetry) {
        if (isShooting) {
            String sequence = PPG ? "PPG" : PGP ? "PGP" : "GPP";

            int neededValue = (sequence.charAt(shootStep) == 'P') ? 1 : 2;
            telemetry.addData("VALUE: ", neededValue);

            char chamber = '\0';
            //note: Searches on the current pos if there is the thing we need
            int currentVal = 0;
            if (actualPos == 'a') currentVal = A;
            else if (actualPos == 'b') currentVal = B;
            else if (actualPos == 'c') currentVal = C;

            if (currentVal == neededValue) {
                chamber = actualPos;
            } else {
                if (A == neededValue) chamber = 'a';
                else if (B == neededValue) chamber = 'b';
                else if (C == neededValue) chamber = 'c';
            }
            if ((neededValue != A && neededValue != B && neededValue != C) && (A != 0 || B != 0 || C != 0)) {//note: no hay valor requerido pero si hay artefactos
                if (A != 0) chamber = 'a';
                else if (B != 0) chamber = 'b';
                else chamber = 'c';
            }
            if (chamber == '\0') { //note:  skipear si no hay artefactos
                shootStep++;
                shootStartTime = System.currentTimeMillis();
            } else { //note: changes the barrel pos
                double targetPos = (chamber == 'a') ? Aout : (chamber == 'b') ? Bout : Cout;
                barril.setPosition(targetPos);
                actualPos = chamber;
                if (System.currentTimeMillis() - shootStartTime >= PATEADOR_ON_TIME) {
                    pateador.setPosition(pateador_on); //fixme
                }
                if (System.currentTimeMillis() - shootStartTime >= PATEADOR_OFF_TIME) {
                    pateador.setPosition(pateador_off); //fixme
                }
                if (System.currentTimeMillis() - shootStartTime >= OUTTAKE_HOLD_TIME_MS) {
                    if (chamber == 'a') A = 0;
                    else if (chamber == 'b') B = 0;
                    else C = 0;
                    shootStep++;
                    if (shootStep >= 3) {
                        shootPow(0);
                        cannon.setPower(0);
                        isShooting = false;
                        piringolaOFF();
                        advanceToPreferredEmpty();
                    }
                    shootStartTime = System.currentTimeMillis();
                }
            }
        } else {
            if (!check) {
//TL --------- INTAKE MODE ------
                // empty chambers return to home
                if ((A == 0 && B == 0 && C == 0)) {
                    barril.setPosition(Ain);
                    actualPos = 'a';
                }

                TestColorSensorMecanism.DetectedColor detected = getDetectedColor(telemetry);
                boolean canIntakeNow = System.currentTimeMillis() - lastIntakeTime >= INTAKE_COOLDOWN_MS;
                if (canIntakeNow &&
                        (detected == TestColorSensorMecanism.DetectedColor.PURPLE ||
                                detected == TestColorSensorMecanism.DetectedColor.GREEN)) {

                    int value = detected == TestColorSensorMecanism.DetectedColor.PURPLE ? 1 : 2;

                    boolean actuallyLoaded = false;

                    if (actualPos == 'a' && A == 0) {
                        A = value;
                        actuallyLoaded = true;
                    } else if (actualPos == 'b' && B == 0) {
                        B = value;
                        actuallyLoaded = true;
                    } else if (actualPos == 'c' && C == 0) {
                        C = value;
                        actuallyLoaded = true;
                    }

                    if (actuallyLoaded) {
                        advanceToPreferredEmpty();
                        lastIntakeTime = System.currentTimeMillis(); // cooldown
                    }
                }
                int currentValue = switch (actualPos) {
                    case 'a' -> A;
                    case 'b' -> B;
                    case 'c' -> C;
                    default -> 0;
                };

                if (currentValue != 0 && (A == 0 || B == 0 || C == 0)) {
                    advanceToPreferredEmpty();
                }
            }
            else {
                boolean canIntakeNow = System.currentTimeMillis() - lastIntakeTime >= INTAKE_COOLDOWN_MS;
                TestColorSensorMecanism.DetectedColor detected = getDetectedColor(telemetry);

                int value = detected == TestColorSensorMecanism.DetectedColor.PURPLE ? 1 : 2;
                boolean actuallyLoaded = false;

                if (checkStep == 0) {
                    barril.setPosition(Ain);
                    actualPos = 'a';
                    if (canIntakeNow && (detected == TestColorSensorMecanism.DetectedColor.PURPLE || detected == TestColorSensorMecanism.DetectedColor.GREEN)){
                        A = value;
                        actuallyLoaded = true;
                    }else if ((System.currentTimeMillis()-lastIntakeTime >= INTAKE_COOLDOWN_MS + NO_INTAKE_COOLDOWN_MS)){
                        A = 0;
                        actuallyLoaded = true;
                    }
                }
                if (checkStep == 1) {
                    barril.setPosition(Bin);
                    actualPos = 'b';
                    if (canIntakeNow && (detected == TestColorSensorMecanism.DetectedColor.PURPLE || detected == TestColorSensorMecanism.DetectedColor.GREEN)){
                        B = value;
                        actuallyLoaded = true;
                    }else if ((System.currentTimeMillis()-lastIntakeTime >= INTAKE_COOLDOWN_MS + NO_INTAKE_COOLDOWN_MS)){
                        B = 0;
                        actuallyLoaded = true;
                    }
                }
                if (checkStep == 2) {
                    barril.setPosition(Cin);
                    actualPos = 'c';
                    if (canIntakeNow && (detected == TestColorSensorMecanism.DetectedColor.PURPLE || detected == TestColorSensorMecanism.DetectedColor.GREEN)){
                        C = value;
                        actuallyLoaded = true;
                    }
                    else if ((System.currentTimeMillis()-lastIntakeTime >= INTAKE_COOLDOWN_MS + NO_INTAKE_COOLDOWN_MS)){
                        C = 0;
                        actuallyLoaded = true;
                    }
                }
                if (shootStep >= 3) {
                    check = false;
                    advanceToPreferredEmpty();
                }
                if (actuallyLoaded) {
                    checkStep ++;
                    lastIntakeTime = System.currentTimeMillis(); // cooldown
                }
            }
        }
    }
    private void advanceToPreferredEmpty()
    {
        if (actualPos == 'a') {
            if (B == 0) {
                barril.setPosition(Bin);
                actualPos = 'b';
            } else if (C == 0) {
                barril.setPosition(Cin);
                actualPos = 'c';
            }
        } else if (actualPos == 'b') {
            if (A == 0) {
                barril.setPosition(Ain);
                actualPos = 'a';
            } else if (C == 0) {
                barril.setPosition(Cin);
                actualPos = 'c';
            }
        } else if (actualPos == 'c') {
            if (A == 0) {
                barril.setPosition(Ain);
                actualPos = 'a';
            } else if (B == 0) {
                barril.setPosition(Bin);
                actualPos = 'b';
            }
        }
    }



    public TestColorSensorMecanism.DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        //TODO colors

        if (normRed < 0.08 && normGreen > 0.1 && normBlue < 0.22) {
            return TestColorSensorMecanism.DetectedColor.GREEN;
        } else if (normRed < 0.2 && normGreen < 0.22 && normBlue > 0.1) {
            return TestColorSensorMecanism.DetectedColor.PURPLE;
        } else {
            return TestColorSensorMecanism.DetectedColor.UNKNOWN;
        }
    }

    public void telem(Telemetry telemetry){
        if (GPP){telemetry.addLine("PATTERN = GPP");telemetry.speak("puto");}
        if (PGP) {telemetry.addLine("PATTERN = PGP");}
        if (PPG) {telemetry.addLine("PATTERN = PPG");}
        if (DESIRED_TAG_ID == 20){ telemetry.addLine("====BLUE TEAM====");}
        if (DESIRED_TAG_ID == 24){ telemetry.addLine("====RED TEAM====");}
        telemetry.addData("A: ",A);
        telemetry.addData("B: ",B);
        telemetry.addData("C: ",C);
        telemetry.addLine("");
        telemetry.addData("Actual Pos: ", actualPos);

        telemetry.addData("POW: ", pow1);
// speak
        telemetry.update();
    }
}