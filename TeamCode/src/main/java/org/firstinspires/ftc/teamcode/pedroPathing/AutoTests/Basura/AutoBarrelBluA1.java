package org.firstinspires.ftc.teamcode.pedroPathing.AutoTests.Basura;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OpMaster.Mecanismos;
import org.firstinspires.ftc.teamcode.pedroPathing.Tests.TestColorSensorMecanism;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
@Disabled
@Autonomous()
public class AutoBarrelBluA1 extends OpMode {
    Mecanismos mecanism = new Mecanismos();
    Mecanismos.DetectedColor detectedColor;
//TL: VARIABLES BARRIL
//Tl: POSICIONES☺
    private static final double  Ain = 0.38;
    private static final double  Bin = 0.46;
    private static final double  Cin = 0.535;
    private static final double  Aout = 0.5;
    private static final double  Bout = 0.57;
    private static final double  Cout = 0.65;
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

    private final long OUTTAKE_HOLD_TIME_MS = 3000;
    private long lastIntakeTime = 0;
    private static final long INTAKE_COOLDOWN_MS = 800;
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    public double timeStamp;

    private final Pose startPose = new Pose(32, 136.5, Math.toRadians(0));      //PT: Start Pose of robot.
    private final Pose startCP = new Pose(35, 125);                             //CP: CONTROL POINT FOR STP-SHP.
    private final Pose obeliskPose = new Pose(60, 132, Math.toRadians(55));         //PT: ROBOT IS SEEING THE OBELISK.
    private final Pose shootPose = new Pose(48, 130, Math.toRadians(180));      //PT: Robot is horizontally seeing the blue goal.
    private final Pose pickIIIPose = new Pose(40, 84, Math.toRadians(180));     //PT: Top (First Set) Blue.
    private final Pose pickedIIIPose = new Pose(14, 84, Math.toRadians(180));   //PT: Top (First Set) Blue already picked.
    private final Pose release = new Pose(10, 70, Math.toRadians(90));          //PT: Release Gate
    private final Pose releaseCP = new Pose(40, 80);
    private final Pose releaseCP2 = new Pose(40, 70);
    private final Pose pickIIPose = new Pose(40, 60, Math.toRadians(180));      //PT: Middle (Second Set) Blue.
    private final Pose pickedIIPose = new Pose(14, 60, Math.toRadians(180));    //PT: Middle (Second Set) Blue already picked.
    private final Pose pickIPose = new Pose(40, 35, Math.toRadians(180));       //PT: Low (First Set) Blue.
    private final Pose pickedIPose = new Pose(14, 35, Math.toRadians(180));     //PT: Low (First Set) Blue already picked.
    private final Pose parkPose = new Pose(14, 110, Math.toRadians(0));         //PT: Robot tangent to blue ramp.

    private Path startpath;
    private PathChain shoot2iii, pickiii, iii2release, iii2shoot, shoot2ii, pickii, ii2shoot, shoot2i, picki, i2shoot, park; //obelisk2IIIBlue

    public void buildPaths() {
        startpath = new Path(new BezierCurve(startPose, startCP, shootPose));
        startpath.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

//        obelisk2IIIBlue = follower.pathBuilder()
//                .addPath(new BezierLine(obeliskPose, pickupIIIPose))
//                .setLinearHeadingInterpolation(obeliskPose.getHeading(), pickupIIIPose.getHeading())
//                .build();

        shoot2iii = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickIIIPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickIIIPose.getHeading())
                .build();

        pickiii = follower.pathBuilder()
                .addPath(new BezierLine(pickIIIPose, pickedIIIPose))
                .setLinearHeadingInterpolation(pickIIIPose.getHeading(), pickedIIIPose.getHeading())
                .build();

        iii2release = follower.pathBuilder()
                .addPath(new BezierCurve(pickIIIPose, releaseCP, releaseCP2, release))
                .setLinearHeadingInterpolation(pickIIIPose.getHeading(), release.getHeading())
                .build();

        iii2shoot = follower.pathBuilder()
                .addPath(new BezierLine(release, shootPose))
                .setLinearHeadingInterpolation(release.getHeading(), shootPose.getHeading())
                .build();

        shoot2ii = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickIIPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickIIPose.getHeading())
                .build();

        pickii = follower.pathBuilder()
                .addPath(new BezierLine(pickIIPose, pickedIIPose))
                .setLinearHeadingInterpolation(pickIIPose.getHeading(), pickedIIPose.getHeading())
                .build();

        ii2shoot = follower.pathBuilder()
                .addPath(new BezierCurve(pickedIIPose, pickIIPose, shootPose))
                .setLinearHeadingInterpolation(pickedIIPose.getHeading(), shootPose.getHeading())
                .build();

        shoot2i = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickIPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickIPose.getHeading())
                .build();

        picki = follower.pathBuilder()
                .addPath(new BezierLine(pickIPose, pickedIPose))
                .setLinearHeadingInterpolation(pickIPose.getHeading(), pickedIPose.getHeading())
                .build();

        i2shoot = follower.pathBuilder()
                .addPath(new BezierLine(pickedIPose, shootPose))
                .setLinearHeadingInterpolation(pickedIPose.getHeading(), shootPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        double actualTime = pathTimer.getElapsedTimeSeconds();

        switch (pathState) {
            case 0:
                follower.followPath(startpath);
                if (!follower.isBusy()){
                    autoShoot();
                    setPathState(1);
                }
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                if (!follower.isBusy() && !isShooting) {
                    follower.followPath(shoot2iii, true);
                    setPathState(2);
                }
                break;
            case 2: //note: INTAKE
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.35);
                    mecanism.intake( 1);
                    follower.followPath(pickiii, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    mecanism.intake(0);
                    follower.followPath(iii2release, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(iii2shoot, true);
                    timeStamp = actualTime;
                    //mecanism.shoot(0.45);
                    setPathState(5);
                }
                break;
            case 5:

                if (!follower.isBusy() && actualTime >= timeStamp + 3) {
                    //mecanism.shoot(0);
                    follower.followPath(shoot2ii, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    mecanism.intake( 1);
                    follower.followPath(pickii, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    mecanism.intake( 0);
                    follower.followPath(ii2shoot, true);
                    timeStamp = actualTime;
                    //mecanism.shoot(0.45);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && actualTime >= timeStamp + 3) {
                    //mecanism.shoot(0);
                    follower.followPath(shoot2i, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    mecanism.intake( 1);

                    follower.followPath(picki, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    mecanism.intake( 0);
                    follower.followPath(i2shoot, true);
                    timeStamp = actualTime;
                    //mecanism.shoot(0.45);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()  && actualTime >= timeStamp + 3) {
                    //mecanism.shoot(0);
                    follower.followPath(park, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    setPathState(-1
                    );
                }
                break;
        }
    }

    
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
//tl: ------CAMERA-------
        List<AprilTagDetection> currentDetections = mecanism.aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((detection.id == 21)) {
                    mecanism.DESIRED_TAG_ID = 21;
                    break;
                }if ((detection.id == 22)) {
                    mecanism.DESIRED_TAG_ID = 22;
                    break;
                }if ((detection.id == 23)) {
                    mecanism.DESIRED_TAG_ID = 23;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        //TL ---------- MODE SELECT ----------
        if (mecanism.DESIRED_TAG_ID == 23) {
            PPG = true;
            PGP = false;
            GPP = false;
        }
        if (mecanism.DESIRED_TAG_ID == 22) {
            PGP = true;
            PPG = false;
            GPP = false;
        }
        if (mecanism.DESIRED_TAG_ID == 21) {
            GPP = true;
            PPG = false;
            PGP = false;
        }
        if (mecanism.DESIRED_TAG_ID != 21 && mecanism.DESIRED_TAG_ID != 22 && mecanism.DESIRED_TAG_ID != 23) {
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
        if (isShooting) {
            String sequence = PPG ? "PPG" : PGP ? "PGP" : "GPP";

            int neededValue = (sequence.charAt(shootStep) == 'P') ? 1 : 2;
            telemetry.addData("VALUE: ", neededValue);

            char chamber = '\0';
            //note: Searches on the current pos if there si the shit we need
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

            if (chamber == '\0') { //note:  skipeao
                //fixme:
                shootStep++;
                shootStartTime = System.currentTimeMillis();
            } else {
                //note: changes the barrel pos
                double targetPos = (chamber == 'a') ? Aout : (chamber == 'b') ? Bout : Cout;
                mecanism.barril.setPosition(targetPos);
                actualPos = chamber;

                if (System.currentTimeMillis() - shootStartTime >= OUTTAKE_HOLD_TIME_MS) {
                    if (chamber == 'a') A = 0;
                    else if (chamber == 'b') B = 0;
                    else C = 0;

                    shootStep++;

                    if (shootStep >= 3) {
                        isShooting = false;
                        advanceToPreferredEmpty();
                    }
                    shootStartTime = System.currentTimeMillis();
                }
            }

            if (shootStep >= 3) {
                isShooting = false;
                advanceToPreferredEmpty();
            }

        } else {
//TL             --------- INTAKE MODE ------
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
                    advanceToPreferredEmpty();                  // rotate immediately
                    lastIntakeTime = System.currentTimeMillis(); // start cooldown so we don't instantly load the same pixel again
                }
            }

// Auto-advance even if we didn't intake (covers manual rotate, after shooting, pixel fell out, etc.)
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
    }
    //tl ----------------MOVE BARREL---------------------
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
    public void autoShoot(){
        if (!isShooting && (PPG || PGP || GPP)) {
            isShooting = true;
            shootStep = 0;
            shootStartTime = System.currentTimeMillis();
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        mecanism.initAll(hardwareMap);


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        mecanism.barril.setPosition(Ain);

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}