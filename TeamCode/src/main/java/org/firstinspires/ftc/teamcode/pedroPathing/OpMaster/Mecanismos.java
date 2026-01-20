package org.firstinspires.ftc.teamcode.pedroPathing.OpMaster;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Tests.TestColorSensorMecanism;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.util.ElapsedTime;
/**
 *>>Control Hub:
 * motores:
 *  ("rightFront")  //0
 *  ("rightRear")    //2
 *  ("leftRear")  //3
 *  ("leftFront")    //1
 * Servos:
 *   barril = 0
 *   intakeR = x
 *   intakeL = 1
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
// bhkbkbkb
public class Mecanismos {
    //Tl:========= INTAKE =========
    public DcMotor intake;
    public  Servo uman;
    public  Servo intaker_L;
    public  Servo intaker_R;
    //TL:======== CANNON ==========
    public DcMotor cannonR;
    public DcMotor cannonL;
    public DcMotor turret;
    //Tl:======== BARRIL ==========
    public Servo barril;
    public Servo pateador;
    NormalizedColorSensor colorSensor;
    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN,
    }
    public static final double  Ain = 0.86;
    public static final double  Bin = 0.79;
    public static final double  Cin = 0.72 ;

    public static final double  Aout = 0.96;
    public static final double  Bout = 0.895;
    public static final double  Cout = 0.83;
    public static final double  pateador_off = 0.5;
    public static final double  pateador_on = 0.46;
    char actualPos = 'a';
    //NOTE: 0 = empty || 1 = PURPLE |
    // | 2 = GREEN
    public int A = 0;
    public int B = 0;
    public int C = 0;
    //TL: MODES
    public boolean PPG = false;
    public boolean PGP = false;
    public boolean GPP = false;
    public boolean NoPattern = false;

    public boolean isShooting = false;
    public int shootStep = 0;
    public long shootStartTime = 0;

    public final long OUTTAKE_HOLD_TIME_MS = 1700;
    public long lastIntakeTime = 0;
    public long INTAKE_COOLDOWN_MS = 800;
    //Tl:       COSOS CHISTOSOS
    public double slowModeMultiplier = 0.3; //Modo slow
    public boolean invertedDrive;
    public boolean RBflag;
    public boolean RB2flag;

    //note    AprilTag search.
    public final double DESIRED_DISTANCE =  48;
    public final double SPEED_GAIN  =  0.02;
    public final double TURN_GAIN   =  0.01;
    public final double MAX_AUTO_SPEED = 0.5;
    public final double MAX_AUTO_TURN  = 0.3;

    public int DESIRED_TAG_ID = 0;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public AprilTagDetection desiredTag = null;

    public boolean targetFound     = false;
    public double  drive           = 0;
    public double  turn            = 0;

    public double  turretP      = 0;
    public double  turretD      = 0;
    public final double Kp   =  0.01  ;
    public final double Kd   =  0.01  ;
    public double lastHeadingError = 0.0;
    public ElapsedTime timer = new ElapsedTime();  // Timer for delta time


//TL: ============= INIT =================
    public void initAll(HardwareMap hwMap){
        pateador = hwMap.get(Servo.class, "pateador");
        intake = hwMap.get(DcMotor.class, "Intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uman = hwMap.get(Servo.class, "uman");
        intaker_L = hwMap.get(Servo.class, "intakerL");
        intaker_R = hwMap.get(Servo.class, "intakerR");


        cannonR = hwMap.get(DcMotor.class, "CannonR");
        cannonL = hwMap.get(DcMotor.class, "CannonL");
        cannonL.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        barril = hwMap.get(Servo.class,"servo");

        colorSensor.setGain(10);

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        uman.setPosition(1.0);
    }
    public void shootPow(double power){
        cannonR.setPower(power);
        cannonL.setPower(power);
    }
    public void intake(double pow){
        intake.setPower(pow);
    }
    public void shootFar(){
        if (!isShooting) {
            shootPow(1.0); //fixme
            isShooting = true;
            shootStep = 0;
            shootStartTime = System.currentTimeMillis();
        }
    }
    public void shoot(){
        if (!isShooting) {
            shootPow(.85); //fixme
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
            barril.setPosition(0.74);
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
            //note: Searches on the current pos if there is the shit we need
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
                shootPow(0.8);
            } else { //note: changes the barrel pos
                double targetPos = (chamber == 'a') ? Aout : (chamber == 'b') ? Bout : Cout;
                barril.setPosition(targetPos);
                actualPos = chamber;
                if (System.currentTimeMillis() - shootStartTime >= 800) {
                    pateador.setPosition(pateador_on); //fixme
                }
                if (System.currentTimeMillis() - shootStartTime >= 1200) {
                    pateador.setPosition(pateador_off); //fixme
                }
                if (System.currentTimeMillis() - shootStartTime >= OUTTAKE_HOLD_TIME_MS) {
                    if (chamber == 'a') A = 0;
                    else if (chamber == 'b') B = 0;
                    else C = 0;

                    shootStep++;
                    shootPow(0.8);
                    if (shootStep >= 3) {
                        shootPow(0);
                        isShooting = false;
                        advanceToPreferredEmpty();
                    }
                    shootStartTime = System.currentTimeMillis();
                }
            }
        } else {
//TL --------- INTAKE MODE ------
            // empty chambers return to home
            if (A == 0 && B == 0 && C == 0) {
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
    }
    private void advanceToPreferredEmpty() {
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
        if (GPP){telemetry.addLine("PATTERN = GPP");}
        if (PGP) {telemetry.addLine("PATTERN = PGP");}
        if (PPG) {telemetry.addLine("PATTERN = PPG");}
        if (DESIRED_TAG_ID == 20){ telemetry.addLine("====BLUE TEAM====");}
        if (DESIRED_TAG_ID == 24){ telemetry.addLine("====RED TEAM====");}
        telemetry.addData("A: ",A);
        telemetry.addData("B: ",B);
        telemetry.addData("C: ",C);
        telemetry.addLine("");

        telemetry.addData("POWER", cannonL.getPower());

        telemetry.addData("Inverted Drive: ",invertedDrive);
        telemetry.update();
    }
}