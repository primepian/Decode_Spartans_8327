package org.firstinspires.ftc.teamcode.pedroPathing.AutoTests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OpMaster.Mecanismos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@Autonomous(name = "BLUE_Auto", group = "Autonomous")
public class Blue_Auto extends OpMode{
    Mecanismos mecanism = new Mecanismos();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private double time_Stamp;

    private final Pose startingPose = new Pose(55.000, 8.000, Math.toRadians(90));            //TL:Path #1
    private final Pose startingPose_CP = new Pose(64.000, 66.000);                            //TL:Path #1
    private final Pose search_pose = new Pose(55.000, 101.000, Math.toRadians(66));           //TL:Path #1

    private final Pose shoot_Pose = new Pose(53.000, 90.000, Math.toRadians(135));           //TL:Path #2 TODO: Shoot fixme:57.000, 105.000, Math.toRadians(145)

    private final Pose fst_itk_pose_CP = new Pose(49.000, 84.000);                            //TL:Path #3
    private final Pose fst_itk_pose = new Pose(50.000, 78.000, Math.toRadians(180));          //TL:Path #3

    private final Pose fst_itk_1 = new Pose(40.000, 78.000, Math.toRadians(180));             //TL:Path #4

    private final Pose fst_itk_2 = new Pose(35.000, 78.000, Math.toRadians(180));             //TL:Path #5

    private final Pose fst_itk_3 = new Pose(20.000, 78.000, Math.toRadians(180));             //TL:Path #6

    private final Pose snd_shoot_CP = new Pose(49.000, 81.000);                               //TL:Path #7 TODO: With SHOOT

    private final Pose snd_itk_pose_CP = new Pose(57.000, 60.000);                            //TL:Path #8
    private final Pose snd_itk_pose = new Pose(56.000, 56.000, Math.toRadians(180));          //TL:Path #8

    private final Pose snd_itk_1 = new Pose(40.000, 56.000, Math.toRadians(180));             //TL:Path #9

    private final Pose snd_itk_2 = new Pose(35.000, 56.000, Math.toRadians(180));             //TL:Path #10

    private final Pose snd_itk_3 = new Pose(20.000, 56.000, Math.toRadians(180));             //TL:Path #11

    private final Pose trd_shoot_CP = new Pose(54.000, 53.000);                               //TL:Path #12 TODO: With SHOOT

    private final Pose trd_itk_pose = new Pose(56.000, 32.000, Math.toRadians(180));          //TL:Path #13

    private final Pose trd_itk_1 = new Pose(47.000, 32.000, Math.toRadians(180));             //TL:Path #14

    private final Pose trd_itk_2 = new Pose(42.000, 32.000, Math.toRadians(180));             //TL:Path #15

    private final Pose trd_itk_3 = new Pose(36.000, 32.000, Math.toRadians(180));             //TL:Path #16

    private final Pose frth_shoot_CP = new Pose(46.000, 91.000);                              //TL:Path #17 TODO: With SHOOT

    private final Pose parking_pose = new Pose(16.000, 103.000, Math.toRadians(180));         //TL:Path #18


    private Path start_path;
    private PathChain snd_path, trd_path, fth_path, fvth_path, sxth_path, svnth_path, egth_path, nnth_path, tenth_path, elvnth_path, twlfth_path, thirtnth_path, frtnth_path, fftnth_path, sxtnth_path, svntnth_path, eightnth_path, nntnth_path;

    public void buildPaths() {

        start_path = new Path(new BezierCurve(startingPose, startingPose_CP, search_pose));
        start_path.setLinearHeadingInterpolation(startingPose.getHeading(), search_pose.getHeading());

        snd_path = follower.pathBuilder()
                .addPath(new BezierLine(search_pose, shoot_Pose))
                .setLinearHeadingInterpolation(search_pose.getHeading(), shoot_Pose.getHeading())
                .build();

        trd_path = follower.pathBuilder()
                .addPath(new BezierCurve(shoot_Pose, fst_itk_pose_CP, fst_itk_pose))
                .setLinearHeadingInterpolation(shoot_Pose.getHeading(), fst_itk_pose.getHeading())
                .build();

        fth_path = follower.pathBuilder()
                .addPath(new BezierLine(fst_itk_pose, fst_itk_1))
                .setLinearHeadingInterpolation(fst_itk_pose.getHeading(), fst_itk_1.getHeading())
                .build();

        fvth_path = follower.pathBuilder()
                .addPath(new BezierLine(fst_itk_1, fst_itk_2))
                .setLinearHeadingInterpolation(fst_itk_1.getHeading(), fst_itk_2.getHeading())
                .build();

        sxth_path = follower.pathBuilder()
                .addPath(new BezierLine(fst_itk_2, fst_itk_3))
                .setLinearHeadingInterpolation(fst_itk_2.getHeading(), fst_itk_3.getHeading())
                .build();

        svnth_path = follower.pathBuilder()
                .addPath(new BezierCurve(fst_itk_3, snd_shoot_CP, shoot_Pose))
                .setLinearHeadingInterpolation(fst_itk_3.getHeading(), shoot_Pose.getHeading())
                .build();

        egth_path = follower.pathBuilder()
                .addPath(new BezierCurve(shoot_Pose, snd_itk_pose_CP, snd_itk_pose))
                .setLinearHeadingInterpolation(shoot_Pose.getHeading(), snd_itk_pose.getHeading())
                .build();

        nnth_path = follower.pathBuilder()
                .addPath(new BezierLine(snd_itk_pose, snd_itk_1))
                .setLinearHeadingInterpolation(snd_itk_pose.getHeading(), snd_itk_1.getHeading())
                .build();

        tenth_path = follower.pathBuilder()
                .addPath(new BezierLine(snd_itk_1, snd_itk_2))
                .setLinearHeadingInterpolation(snd_itk_1.getHeading(), snd_itk_2.getHeading())
                .build();

        elvnth_path = follower.pathBuilder()
                .addPath(new BezierLine(snd_itk_2, snd_itk_3))
                .setLinearHeadingInterpolation(snd_itk_2.getHeading(), snd_itk_3.getHeading())
                .build();

        twlfth_path = follower.pathBuilder()
                .addPath(new BezierCurve(snd_itk_3, trd_shoot_CP, shoot_Pose))
                .setLinearHeadingInterpolation(snd_itk_3.getHeading(), shoot_Pose.getHeading())
                .build();

        thirtnth_path = follower.pathBuilder()
                .addPath(new BezierLine(shoot_Pose, trd_itk_pose))
                .setLinearHeadingInterpolation(shoot_Pose.getHeading(), trd_itk_pose.getHeading())
                .build();

        frtnth_path = follower.pathBuilder()
                .addPath(new BezierLine(trd_itk_pose, trd_itk_1))
                .setLinearHeadingInterpolation(trd_itk_pose.getHeading(), trd_itk_1.getHeading())
                .build();

        fftnth_path = follower.pathBuilder()
                .addPath(new BezierLine(trd_itk_1, trd_itk_2))
                .setLinearHeadingInterpolation(trd_itk_1.getHeading(), trd_itk_2.getHeading())
                .build();

        sxtnth_path = follower.pathBuilder()
                .addPath(new BezierLine(trd_itk_2, trd_itk_3))
                .setLinearHeadingInterpolation(trd_itk_2.getHeading(), trd_itk_3.getHeading())
                .build();

        svntnth_path = follower.pathBuilder()
                .addPath(new BezierCurve(trd_itk_3, frth_shoot_CP, shoot_Pose))
                .setLinearHeadingInterpolation(trd_itk_3.getHeading(), shoot_Pose.getHeading())
                .build();

        eightnth_path = follower.pathBuilder()
                .addPath(new BezierLine(shoot_Pose, parking_pose))
                .setLinearHeadingInterpolation(shoot_Pose.getHeading(), parking_pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        double actual_time = pathTimer.getElapsedTimeSeconds();

        switch (pathState) {
            case 0: //start to obelisk
                follower.followPath(start_path);
                time_Stamp = actual_time;
                setPathState(1);
                if(mecanism.A == 0){mecanism.A = 1;}
                if(mecanism.B == 0){mecanism.B = 1;}
                if(mecanism.C == 0){mecanism.C = 2;}
                break;
            case 1://obelisk to shoot
                if (mecanism.DESIRED_TAG_ID == 21){mecanism.GPP = true;}
                if (mecanism.DESIRED_TAG_ID == 22){mecanism.PGP = true;}
                if (mecanism.DESIRED_TAG_ID == 23){mecanism.PPG = true;}
                if ((!follower.isBusy() && actual_time >= time_Stamp + 3) || (!follower.isBusy() && (mecanism.PPG || mecanism.GPP || mecanism.PGP) )) {
                    follower.followPath(snd_path,true);
                    if (follower.getPose().getY() < 91){
                        mecanism.shootA();                                                           //TL:SHOOT
                        setPathState(2);
                    }
                }
                break;
                
            case 2:
                if (!follower.isBusy() && !mecanism.isShooting) { // && !mecanism.isShooting
                    follower.followPath(trd_path, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.7); //fixme
                    mecanism.intake(-8);
                    follower.followPath(fth_path, true);                                    //TL:FIRST INTAKE, 1
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    pathState = 5;
                }
                break;
            case 5:
                if (actual_time >= time_Stamp + 0.7) {
                    mecanism.intakerON();
                    if (actual_time >= time_Stamp + 1.5) {
                        mecanism.intakerOFF();
                        mecanism.A = 1;
                        mecanism.B = 0;
                        mecanism.C = 0;
                        follower.followPath(fvth_path, true);                                   //TL:FIRST INTAKE, 2
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    pathState = 7;
                }
                break;
            case 7:
                if (actual_time >= time_Stamp + 0.7) {
                    mecanism.intakerON();
                    if (actual_time >= time_Stamp + 1.5) {
                        mecanism.intakerOFF();
                        mecanism.B = 1;
                        mecanism.C = 0;
                        follower.followPath(sxth_path, true);                                   //TL:FIRST INTAKE, 3
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    pathState = 9;
                }
                break;
            case 9:
                if (actual_time >= time_Stamp + 0.7) {
                    mecanism.intakerON();
                    if (actual_time >= time_Stamp + 1.5) {
                        mecanism.intakerON();
                        mecanism.C = 2;
                        follower.setMaxPower(1);
                        mecanism.intake(0);
                        follower.followPath(svnth_path, true);
                        if (follower.getPose().getY() > 89) {
                            mecanism.shootA();                                                           //TODO:SHOOT
                            setPathState(10);
                            }
                        }
                    }
                break;
            case 10:
                if (!follower.isBusy() && !mecanism.isShooting) {
                    follower.followPath(egth_path, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    mecanism.intake(-1);
                    follower.followPath(nnth_path, true);                                   //TL:SECOND INTAKE, 1
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    pathState = 13;
                }
                break;
            case 13:
                if (actual_time >= time_Stamp + 0.7) {
                    mecanism.intakerON();
                    if (actual_time >= time_Stamp + 1.5) {
                        mecanism.intakerOFF();
                        mecanism.A = 1;
                        mecanism.B = 0;
                        mecanism.C = 0;
                        follower.followPath(tenth_path, true);                                  //TL:SECOND INTAKE, 2
                        setPathState(14);
                    }
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    pathState = 15;
                }
                break;
            case 15:
                if (actual_time >= time_Stamp + 0.7) {
                    mecanism.intakerON();
                    if (actual_time >= time_Stamp + 1.5) {
                        mecanism.intakerOFF();
                        mecanism.B = 2;
                        mecanism.C = 0;
                        follower.followPath(elvnth_path, true);                                 //TL:SECOND INTAKE, 3
                        setPathState(16);
                    }
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    time_Stamp = actual_time;
                    pathState = 17;
                }
                break;
            case 17:
                if (actual_time >= time_Stamp + 0.7) {
                    mecanism.intakerON();
                    if (actual_time >= time_Stamp + 1.5) {
                        mecanism.intakerOFF();
                        mecanism.C = 1;
                        follower.setMaxPower(1);
                        mecanism.intake(0);
                        follower.followPath(twlfth_path, true);
                         if (follower.getPose().getY() > 89) {
                            mecanism.shootA();                                                           //TODO:SHOOT
                            setPathState(18);
                         }
                    }
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        mecanism.G28();
        mecanism.shootingandIntakeAuto(telemetry);

        //TL: APRIL TAG DETECTION

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
        telemetry.addData("pathTimer: ", pathTimer);
        telemetry.addData("DESIRED_TAG_ID: ", mecanism.DESIRED_TAG_ID);

        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        mecanism.initAll(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startingPose);

    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}