package org.firstinspires.ftc.teamcode.pedroPathing.AutoTests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto_Pos1_Rev1", group = "Autonomous")
public class Auto_Pos1_Rev1 extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startingPose = new Pose(56.000, 9.000, Math.toRadians(90));            //TL:Path #1
    private final Pose startingPose_CP = new Pose(64.000, 66.000);                            //TL:Path #1
    private final Pose search_pose = new Pose(55.000, 101.000, Math.toRadians(66));           //TL:Path #1

    private final Pose shoot_Pose = new Pose(55.000, 101.000, Math.toRadians(139));          //TL:Path #2

    private final Pose fst_itk_pose_CP = new Pose(49.000, 84.000);                            //TL:Path #3
    private final Pose fst_itk_pose = new Pose(40.000, 84.000, Math.toRadians(180));          //TL:Path #3

    //fixme:

    private final Pose fst_itk = new Pose(19.000, 84.000, Math.toRadians(180));               //TL:Path #4

    private final Pose hatch_CP = new Pose(56.000, 80.000);                                   //TL:Path #5
    private final Pose hatch = new Pose(17.000, 75.000, Math.toRadians(0));                   //TL:Path #5

    private final Pose snd_shoot_CP = new Pose(48.000, 76.000);                               //TL:Path #6

    //fixme:

    private final Pose


    private Path start_path;
    private PathChain snd_path, trd_path, fth_path, fvth_path, sxth_path, svnth_path, egth_path, nnth_path;

    public void buildPaths() {

        start_path = new Path(new BezierCurve(startingPose, startingPose_CP, shoot_Pose));                 //TL:Path #1
        start_path.setLinearHeadingInterpolation(startingPose.getHeading(), shoot_Pose.getHeading());      //TL:Path #1

        snd_path = follower.pathBuilder()
                .addPath(new BezierLine(shoot_Pose, search_pose))
                .setLinearHeadingInterpolation(shoot_Pose.getHeading(), search_pose.getHeading())
                .build();

        trd_path = follower.pathBuilder()
                .addPath(new BezierCurve(search_pose, fst_itk_pose_CP, fst_itk_pose))
                .setLinearHeadingInterpolation(search_pose.getHeading(), fst_itk_pose.getHeading())
                .build();


        fth_path = follower.pathBuilder()
                .addPath(new BezierLine(fst_itk_pose, fst_itk))
                .setLinearHeadingInterpolation(fst_itk_pose.getHeading(), fst_itk.getHeading())
                .build();

        fvth_path = follower.pathBuilder()                                                          //TL:Path #5
                .addPath(new BezierCurve(fst_itk, hatch_CP, hatch))
                .setLinearHeadingInterpolation(fst_itk.getHeading(), hatch.getHeading())
                .build();

        sxth_path = follower.pathBuilder()
                .addPath(new BezierCurve(hatch, snd_shoot_CP, shoot_Pose))
                .setLinearHeadingInterpolation(hatch.getHeading(), shoot_Pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(start_path);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(snd_path,true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(trd_path, true);
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                }
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

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

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