package org.firstinspires.ftc.teamcode.pedroPathing.OpMaster.MecanismTests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OpMaster.Mecanismos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.function.Supplier;
/*
* ======GPAD 1=====
* DRIVE [JOYSTICKS]
* SLOW MODE [L_TRIGGER]
* INTAKE [R_TRIGGER]
* SEARCH APRILTAGS [LEFT BUMPER]
*
* =====GPAD 2======
* BLUE APRILTAG [LEFT STICK BUTTON]
* RED APRILTAG [RIGHT STICK BUTTON]
* CANNON [RIGHT TRIGGER]
*
*/
@Configurable
@TeleOp
@Disabled
public class TeleOp_Can_In extends OpMode {
    Mecanismos mecanism = new Mecanismos();
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;


    // Tl: variables chistosas
    boolean GPP = false;
    boolean PGP = false;
    boolean PPG = false;

    @Override
    public void init() {
        mecanism.initAll(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


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
                    telemetry.addData("TAG FOUND!!!", "Tag ID %d",detection.id);
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
        if (gamepad2.left_stick_button)  {mecanism.DESIRED_TAG_ID = 20; telemetry.addLine("====BLUE TEAM====");}    //NOTE: BLUE TEAM
        if (gamepad2.right_stick_button) {mecanism.DESIRED_TAG_ID = 24; telemetry.addLine("====RED TEAM====");}    //NOTE: RED TEAM

//  TL: INVERT DRIVE    {GPAD_1}
        boolean currentRB = gamepad1.right_bumper;
        if (currentRB && !mecanism.RBflag) {
            mecanism.invertedDrive = !mecanism.invertedDrive;
        }
        mecanism.RBflag = currentRB;

//  TL: INTAKE      {GPAD_1}
//        if (gamepad1.right_trigger > 0.0){
//            inTake(1);
//        }if (gamepad1.left_trigger > 0.0){
//            inTake(-1);
//        }
//        else {
//            mecanism.intake(0);
//        }

//  TL: POS. SHOOT  {GPAD_1}
//  TL: BARRIL      {GPAD_2}
//  TL: CANNON      {GPAD_2}
        if (gamepad2.right_trigger > 0.0){
            shoot(0.35);
        } if (gamepad2.left_trigger > 0.0){
            shoot(0.45);
        } else {
            shoot(0);
        }
//  TL: CAMBIO DE MODO [GPP] [PGP] [PPG]    {GPAD_2}
//        if (gamepad2.dpad_right){GPP = true;}
//        if (gamepad2.dpad_up){PGP = true;}
        if (gamepad2.dpad_left){PPG = true;}

        telem();
    }
    public void telem(){
        telemetry.addData("Inverted Drive: ",mecanism.invertedDrive);
        telemetry.update();
    }

//  NOTE: ======MECANISM STUFF=======
    public void shoot(double power){
        mecanism.cannonR.setPower(power);
        mecanism.cannonL.setPower(power);
    }
}