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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.IllegalFormatCodePointException;
import java.util.List;
import java.util.function.Supplier;
/*
 * ======GPAD 1=====
 * DRIVE [JOYSTICKS]
 * SLOW MODE [L_TRIGGER]
 * INTAKE [R_TRIGGER]
 * -INTAKE [B]
 *
 * SEARCH APRILTAGS [LEFT BUMPER]
 *
 * =====GPAD 2======
 * BLUE APRILTAG [LEFT STICK BUTTON]
 * RED APRILTAG [RIGHT STICK BUTTON]
 * CANNON NEAR [LEFT TRIGGER]
 * CANNON FAR [RIGHT TRIGGER]
 * CHECK [Y]
 * POWERS [A,B,X]
 * SELECT PATTERN [DPADS]
 *
 */

@Configurable
@TeleOp(name = "TeleOpMaster")
public class TeleOpMasterV2 extends OpMode {
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
        mecanism.pateador.setPosition(Mecanismos.pateador_off);

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

//tl: drive / inverted drive
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
        }

//tl: April Tags
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

//tl:  Automatic Shoot Positioning
        if (gamepad1.left_bumper && mecanism.targetFound) {
            double  rangeError      = (mecanism.desiredTag.ftcPose.range - mecanism.DESIRED_DISTANCE);
            double  headingError    = mecanism.desiredTag.ftcPose.bearing;

            mecanism.drive  = Range.clip(rangeError * mecanism.SPEED_GAIN, -mecanism.MAX_AUTO_SPEED, mecanism.MAX_AUTO_SPEED);
            mecanism.turn   = Range.clip(headingError * mecanism.TURN_GAIN, -mecanism.MAX_AUTO_TURN, mecanism.MAX_AUTO_TURN) ;
//mecanism.drive
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x * 0.2,
                    mecanism.turn,
                    true
            );
        }

//  TL: INTAKE      {GPAD_1}
        if (gamepad1.right_trigger > 0.0){mecanism.intake( -0.6);}//NOTE: Intake
        else if (gamepad1.b){mecanism.intake( 0.5);
        } //NOTE: ESCUPELUPE
        else {
            mecanism.intake( 0);}

        if (gamepad1.right_bumper) {mecanism.intakerON();
        } else {mecanism.intakerOFF();
        }//NOTE: DESTAPACAÑOS
        if (gamepad2.back){mecanism.cannon.setPower(-1);}

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
        if (gamepad2.right_trigger > 0.1f) {
            mecanism.shoot();
        } //NOTE: DISPARAR
        if (gamepad2.left_trigger > 0.1f) {
            Mecanismos.pow1 = Mecanismos.POW_CERCA;
            mecanism.shootNear();
        }//NOTE: SHOOT NEAR
        if (gamepad2.right_bumper){
            mecanism.shootPow(0);
            mecanism.cannon.setPower(0);
            mecanism.pateador.setPosition(Mecanismos.pateador_off);
            mecanism.isShooting = false;
        } //NOTE: APAGAR CAÑON

        //  NOTE: [LB] ACTUAL TO 1
        boolean currentRB2 = gamepad2.left_bumper;
        if (currentRB2 && !mecanism.RB2flag){
            if (mecanism.actualPos == 'a' && mecanism.A == 0){mecanism.A = 1;}
            else if (mecanism.actualPos == 'b' && mecanism.B == 0){mecanism.B = 1;}
            else if (mecanism.actualPos == 'c' && mecanism.C == 0){mecanism.C = 1;}
            mecanism.advanceToPreferredEmpty();
        }
        mecanism.RB2flag = currentRB2;

        if (gamepad2.xWasPressed()){
            mecanism.A = 0;
            mecanism.B = 0;
            mecanism.C = 0;
        }

        //NOTE: a,b,x TO set powers
        if (gamepad2.a){
            Mecanismos.pow1 = Mecanismos.POW_LEJOS;
        }
        if (gamepad2.b){
            Mecanismos.pow1 = Mecanismos.POW_MEDIO;
        }

        if (gamepad2.yWasReleased()){
            mecanism.check = true;
            mecanism.checkStep = 0;
            mecanism.lastIntakeTime = System.currentTimeMillis();
            telemetry.speak("CHECKING");
        } //note: check


        mecanism.G28();
        mecanism.shootingandIntake(telemetry);
        mecanism.telem(telemetry);
    }
}