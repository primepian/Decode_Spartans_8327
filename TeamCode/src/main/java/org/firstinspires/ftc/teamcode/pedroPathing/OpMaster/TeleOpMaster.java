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
 * SELECT PATTERN [DPADS]
 * HUMAN MODE [A]
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
    @Override
    public void init() {
        mecanism.initAll(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//       start positions
        mecanism.pateador.setPosition(Mecanismos.pateador_off); //fixme

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
                if (gamepad1.left_trigger == 0.0) follower.setTeleOpDrive(

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
                if (gamepad1.left_trigger == 0.0) follower.setTeleOpDrive(
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
        } //tl: drive / inverted drive

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
        } //tl: April Tags

        if (gamepad1.left_bumper && mecanism.targetFound) {
            double  rangeError      = (mecanism.desiredTag.ftcPose.range - mecanism.DESIRED_DISTANCE);
            double  headingError    = mecanism.desiredTag.ftcPose.bearing;

            mecanism.drive  = Range.clip(rangeError * mecanism.SPEED_GAIN, -mecanism.MAX_AUTO_SPEED, mecanism.MAX_AUTO_SPEED);
            mecanism.turn   = Range.clip(headingError * mecanism.TURN_GAIN, -mecanism.MAX_AUTO_TURN, mecanism.MAX_AUTO_TURN) ;

            follower.setTeleOpDrive(
                    mecanism.drive,
                    -gamepad1.left_stick_x * 0.2,
                    mecanism.turn,
                    true
            );
        }   //tl: Automatic Shoot Positioning

        //  TL: INVERT DRIVE    {GPAD_1}
//        boolean currentRB = gamepad1.right_bumper;
//        if (currentRB && !mecanism.RBflag) {
//            mecanism.invertedDrive = !mecanism.invertedDrive;
//        }
//        mecanism.RBflag = currentRB;

        //  TL: INTAKE      {GPAD_1}
        if (gamepad2.dpad_down){
            mecanism.uman.setPosition(0.18);
            mecanism.intakerON();
            mecanism.INTAKE_COOLDOWN_MS = 800;
        }//NOTE: HUMAN
        else{
            mecanism.uman.setPosition(0.27);
            mecanism.intakerOFF();
            mecanism.INTAKE_COOLDOWN_MS = 300;
        }
        if (gamepad1.right_bumper){
            mecanism.intakerON();
        } else {
            mecanism.intakerOFF();
        }//NOTE: DESTAPACAÑOS
        if (gamepad1.right_trigger > 0.0){
            mecanism.intake( -0.7);}//NOTE: TRAGATODO
        else if (gamepad1.b){mecanism.intake( 0.5);
        } //NOTE: ESCUPELUPE
        else {
            mecanism.intake( 0);}

//  Tl: CHOOSE TEAM FOR SHOOTING POSE   {GPAD_2}
        if (gamepad2.left_stick_button)  {mecanism.DESIRED_TAG_ID = 20;}    //NOTE: BLUE TEAM
        if (gamepad2.right_stick_button) {mecanism.DESIRED_TAG_ID = 24;}    //NOTE: RED TEAM

//TL ---------- MODE SELECT ----------
        if (gamepad2.dpad_right) {
            mecanism.PPG = true;
            mecanism.PGP = false;
            mecanism.GPP = false;
        }
        if (gamepad2.dpad_up) {
            mecanism.PGP = true;
            mecanism.PPG = false;
            mecanism.GPP = false;
        }
        if (gamepad2.dpad_left) {
            mecanism.GPP = true;
            mecanism.PPG = false;
            mecanism.PGP = false;
        }

//tl:---------- CANNON / BARREL -----------
        if (gamepad2.right_trigger > 0.1f){
            mecanism.shoot();
        } //NOTE: DISPARAR
        if (gamepad2.right_bumper){
            mecanism.shootPow(0);
            mecanism.pateador.setPosition(Mecanismos.pateador_off);
            mecanism.isShooting = false;
        } //NOTE: APAGAR CAÑON

        //  NOTE: [LB] ACTUAL TO 1
        boolean currentRB2 = gamepad2.left_bumper;
        if (currentRB2 && !mecanism.RB2flag){
            if (mecanism.actualPos == 'a' && mecanism.A == 0){mecanism.A = 1;}
            else if (mecanism.actualPos == 'b' && mecanism.B == 0){mecanism.B = 1;}
            else if (mecanism.actualPos == 'c' && mecanism.C == 0){mecanism.C = 1;}
        }
        mecanism.RB2flag = currentRB2;

        if (gamepad2.left_trigger > 0.1f) {
            mecanism.shootPow(Mecanismos.pow1);
            
        }//NOTE: SHOOT POW 1

        if (gamepad2.y) {
            mecanism.A = 0;
            mecanism.B = 0;
            mecanism.C = 0;
        } //NOTE: ALL TO 0

        boolean currentBack = gamepad2.back;
    if (currentBack && !mecanism.Backflag){
            mecanism.Back = !mecanism.Back;
        }
        mecanism.Backflag = currentBack;

        //NOTE: a,b,x TO 0
        if (gamepad2.a && mecanism.Back){
            Mecanismos.pow1 = 0.8;
            Mecanismos.pow2 = 0.83;
            Mecanismos.pow3 = 0.81;
        }
        boolean currentb = gamepad2.b;
        if ((currentb && mecanism.bflag) && mecanism.Back){
            Mecanismos.pow1 = Mecanismos.pow1 + 0.02;
            Mecanismos.pow2 = Mecanismos.pow2 + 0.02;
            Mecanismos.pow3 = Mecanismos.pow3 + 0.02;
        }
        mecanism.bflag = currentb;

        boolean currentx = gamepad2.x;
        if ((currentx && mecanism.cflag) && mecanism.Back){
            Mecanismos.pow1 = Mecanismos.pow1 - 0.02;
            Mecanismos.pow2 = Mecanismos.pow2 - 0.02;
            Mecanismos.pow3 = Mecanismos.pow3 - 0.02;
        }
        mecanism.cflag = currentx;

        //sin back
        if (gamepad2.a && !mecanism.Back){
            Mecanismos.pow1 = 0.68;
            Mecanismos.pow2 = 0.65;
            Mecanismos.pow3 = 0.65;
        }
        if (gamepad2.b && !mecanism.Back){
            Mecanismos.pow1 = 0.8;
            Mecanismos.pow2 = 0.82;
            Mecanismos.pow3 = 0.81;
        }
        if (gamepad2.x && !mecanism.Back){
            Mecanismos.pow1 = 0.86;
            Mecanismos.pow2 = 0.87;
            Mecanismos.pow3 = 0.87;
        }

        mecanism.G28();
        mecanism.shootingandIntake(telemetry);
        mecanism.telem(telemetry);
    }
}