package org.firstinspires.ftc.teamcode.pedroPathing.OpMaster;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tests.TestColorSensorMecanism;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.function.Supplier;
/*
 * ======GPAD 1=====
 * DRIVE [JOYSTICKS]
 * SLOW MODE [L_TRIGGER]
 * INTAKE [R_TRIGGER]
 * -INTAKE [B]
 * INVERT DRIVE [RIGHT BUMPER]
 * SEARCH APRILTAGS [LEFT BUMPER]
 *
 * =====GPAD 2======
 * BLUE APRILTAG [LEFT STICK BUTTON]
 * RED APRILTAG [RIGHT STICK BUTTON]
 * CANNON NEAR [RIGHT TRIGGER]
 * CANNON FAR [LEFT TRIGGER]
 */

@Configurable
@TeleOp
public class TeleOpMaster extends OpMode {
    Mecanismos mecanism = new Mecanismos();
    Mecanismos.DetectedColor detectedColor;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
//TL: BARREL POSITIONS
    private static final double  Ain = 0.41;
    private static final double  Bin = 0.495;
    private static final double  Cin = 0.565;
    private static final double  Aout = 0.53;
    private static final double  Bout = 0.6;
    private static final double  Cout = 0.68;
    char actualPos = 'a';
    //NOTE: 0 = empty || 1 = PURPLE || 2 = GREEN
    int A = 0;
    int B = 0;
    int C = 0;
    //TL: MODES
    private boolean PPG = false;
    private boolean PGP = false;
    private boolean GPP = false;

    private boolean isShooting = false;
    private int shootStep = 0;
    private long shootStartTime = 0;

    private final long OUTTAKE_HOLD_TIME_MS = 2000;
    private long lastIntakeTime = 0;
    private static final long INTAKE_COOLDOWN_MS = 800;


    @Override
    public void init() {
        mecanism.initAll(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//       start positions
        mecanism.barril.setPosition(Ain);
        mecanism.pateador.setPosition(0.475); //fixme

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();
        mecanism.targetFound = false;
        mecanism.desiredTag  = null;

        if (!automatedDrive) {//  TL: DRIVE {GPAD_1}
            if (!mecanism.invertedDrive) {
                if (gamepad1.left_trigger == 0.0 && !gamepad1.x) follower.setTeleOpDrive(

                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true // Robot Centric
                );
                else follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * mecanism.slowModeMultiplier,
                        -gamepad1.left_stick_x * mecanism.slowModeMultiplier,
                        -gamepad1.right_stick_x * mecanism.slowModeMultiplier,
                        true // Robot Centric
                );
            } else {
                if (gamepad1.left_trigger == 0.0 && !gamepad1.x) follower.setTeleOpDrive(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true // Robot Centric
                );
                else follower.setTeleOpDrive(
                        gamepad1.left_stick_y * mecanism.slowModeMultiplier,
                        gamepad1.left_stick_x * mecanism.slowModeMultiplier,
                        -gamepad1.right_stick_x * mecanism.slowModeMultiplier,
                        true // Robot Centric
                );
            }
        }

        List<AprilTagDetection> currentDetections = mecanism.aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((detection.id == mecanism.DESIRED_TAG_ID)) {
                    mecanism.targetFound = true;
                    mecanism.desiredTag = detection;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if (gamepad1.left_bumper && mecanism.targetFound) {
            double  rangeError      = (mecanism.desiredTag.ftcPose.range - mecanism.DESIRED_DISTANCE);
            double  headingError    = mecanism.desiredTag.ftcPose.bearing;

            mecanism.drive  = Range.clip(rangeError * mecanism.SPEED_GAIN, -mecanism.MAX_AUTO_SPEED, mecanism.MAX_AUTO_SPEED);
            mecanism.turn   = Range.clip(headingError * mecanism.TURN_GAIN, -mecanism.MAX_AUTO_TURN, mecanism.MAX_AUTO_TURN) ;

            follower.setTeleOpDrive(
                    mecanism.drive,
                    -gamepad1.left_stick_x * 0.3,
                    mecanism.turn,
                    true
            );
        }

//  Tl: CHOOSE TEAM FOR SHOOTING POSE   {GPAD_2}
        if (gamepad2.left_stick_button)  {mecanism.DESIRED_TAG_ID = 20;}    //NOTE: BLUE TEAM
        if (gamepad2.right_stick_button) {mecanism.DESIRED_TAG_ID = 24;}    //NOTE: RED TEAM

//  TL: INVERT DRIVE    {GPAD_1}
        boolean currentRB = gamepad1.right_bumper;
        if (currentRB && !mecanism.RBflag) {
            mecanism.invertedDrive = !mecanism.invertedDrive;
        }
        mecanism.RBflag = currentRB;

//  TL: INTAKE      {GPAD_1}
        if (gamepad1.right_trigger > 0.0){
            mecanism.intake( 1);
        } if (gamepad1.b){
            mecanism.intake( -1);
        } else {
            mecanism.intake( 0);
        }


//TL ================= BARRIL ================== {GPAD_2}
//TL ---------- MODE SELECT ----------
        if (gamepad2.dpad_right) {
            PPG = true;
            PGP = false;
            GPP = false;
        }
        if (gamepad2.dpad_up) {
            PGP = true;
            PPG = false;
            GPP = false;
        }
        if (gamepad2.dpad_left) {
            GPP = true;
            PPG = false;
            PGP = false;
        }
        if (gamepad2.dpad_down) {
            PPG = PGP = GPP = false;
        }
//TL  ---------- EMPTY -> G28 ----------
        if (A == 0 && B == 0 && C == 0) {
            mecanism.barril.setPosition(Ain);
            actualPos = 'a';
            isShooting = false;
        }
        if (A != 0 && B != 0 && C != 0){
            mecanism.barril.setPosition(Aout);
            actualPos = 'a';
        }
//TL ---------- CANNON / AUTOMATIC ----------

        if (gamepad2.right_trigger > 0.1f && !isShooting && (PPG || PGP || GPP)) {
            mecanism.shoot(1.0); //fixme
            isShooting = true;
            shootStep = 0;
            shootStartTime = System.currentTimeMillis();

        }

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
            } else{
                if (A == neededValue) chamber = 'a';
                else if (B == neededValue) chamber = 'b';
                else if (C == neededValue) chamber = 'c';
            }
            if ((neededValue != A && neededValue != B && neededValue != C ) && (A!=0 || B!=0 || C!=0)){//note: no hay valor requerido pero si hay artefactos
                if (A!=0) chamber = 'a';
                else if (B!=0) chamber = 'b';
                else chamber = 'c';
            }
            if (chamber == '\0') { //note:  skipear si no hay artefactos
                shootStep++;
                shootStartTime = System.currentTimeMillis();
            }
            else { //note: changes the barrel pos
                double targetPos = (chamber == 'a') ? Aout : (chamber == 'b') ? Bout : Cout;
                mecanism.barril.setPosition(targetPos);
                actualPos = chamber;
                if (System.currentTimeMillis() - shootStartTime >= 500){
                    mecanism.pateador.setPosition(0.33); //fixme
                }
                if (System.currentTimeMillis() - shootStartTime >= 1000){
                    mecanism.pateador.setPosition(0.475); //fixme
                }
                if (System.currentTimeMillis() - shootStartTime >= OUTTAKE_HOLD_TIME_MS) {
                    if (chamber == 'a') A = 0;
                    else if (chamber == 'b') B = 0;
                    else C = 0;

                    shootStep++;

                    if (shootStep >= 3) {
                        mecanism.shoot(0);
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
                mecanism.barril.setPosition(Ain);
                actualPos = 'a';
            }

            TestColorSensorMecanism.DetectedColor detected = mecanism.getDetectedColor(telemetry);

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

// Auto-advance even if we didn't intake
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
        telemetry.addData("A: ",A);
        telemetry.addData("B: ",B);
        telemetry.addData("C: ",C);
        telemetryM.addLine("");
        mecanism.telem(telemetry);
    }
    public void autoShoot(){
        if (!isShooting && (PPG || PGP || GPP)) {
            isShooting = true;
            shootStep = 0;
            shootStartTime = System.currentTimeMillis();
        }
    }

//tl ----------------MOVE BARREL------------------
    private void advanceToPreferredEmpty() {
        if (actualPos == 'a') {
            if (B == 0) {
                mecanism.barril.setPosition(Bin);
                actualPos = 'b';
            } else if (C == 0) {
                mecanism.barril.setPosition(Cin);
                actualPos = 'c';
            }
        } else if (actualPos == 'b') {
            if (A == 0) {
                mecanism.barril.setPosition(Ain);
                actualPos = 'a';
            } else if (C == 0) {
                mecanism.barril.setPosition(Cin);
                actualPos = 'c';
            }
        } else if (actualPos == 'c') {
            if (A == 0) {
                mecanism.barril.setPosition(Ain);
                actualPos = 'a';
            } else if (B == 0) {
                mecanism.barril.setPosition(Bin);
                actualPos = 'b';
            }
        }
    }
}
