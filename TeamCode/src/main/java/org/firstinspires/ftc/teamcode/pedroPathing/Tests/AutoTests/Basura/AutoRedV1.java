package org.firstinspires.ftc.teamcode.pedroPathing.Tests.AutoTests.Basura;

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

@Autonomous(group = "Tests")
@Disabled
public class AutoRedV1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(32, 135.5, Math.toRadians(270)).mirror(); //PT: Start Pose of robot.
    private final Pose controlPoint = new Pose(35, 118).mirror(); //CP: CONTROL POINT FOR STP-SHP.
    //    private final Pose obeliskPose = new Pose(48, 120, Math.toRadians(50)); //PT: ROBOT IS SEEING THE OBELISK.
    private final Pose shootPose = new Pose(48, 120, Math.toRadians(150)).mirror(); //PT: Robot is diagonally seeing the blue goal.
    private final Pose pickupIIIPose = new Pose(48, 84, Math.toRadians(180)).mirror(); //PT: Top (First Set) Blue.
    private final Pose pickedIIIPose = new Pose(17, 84, Math.toRadians(180)).mirror(); //PT: Top (First Set) Blue already picked.
    private final Pose pickupIIPose = new Pose(48, 60, Math.toRadians(180)).mirror(); //PT: Middle (Second Set) Blue.
    private final Pose pickedIIPose = new Pose(17, 60, Math.toRadians(180)).mirror(); //PT: Middle (Second Set) Blue already picked.
    private final Pose controlPoint2 = new Pose(60, 60).mirror(); //CP: CONTROL POINT FOR III TO SHP.
    private final Pose pickupIPose = new Pose(45, 35, Math.toRadians(180)).mirror(); //PT: Low (First Set) Blue.
    private final Pose pickedIPose = new Pose(18, 35, Math.toRadians(180)).mirror(); //PT: Low (First Set) Blue already picked.
    private final Pose parkPose = new Pose(15, 85, Math.toRadians(0)).mirror();//PT: Robot tangent to blue ramp.

    private Path startpath;
    private PathChain shoot2iii, pickiii, iii2shoot, shoot2ii, pickii, ii2shoot, shoot2i, picki, i2shoot, park; //obelisk2IIIBlue

    public void buildPaths() {
        startpath = new Path(new BezierCurve(startPose, controlPoint, shootPose));
        startpath.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

//        obelisk2IIIBlue = follower.pathBuilder()
//                .addPath(new BezierLine(obeliskPose, pickupIIIPose))
//                .setLinearHeadingInterpolation(obeliskPose.getHeading(), pickupIIIPose.getHeading())
//                .build();

        shoot2iii = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupIIIPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupIIIPose.getHeading())
                .build();

        pickiii = follower.pathBuilder()
                .addPath(new BezierLine(pickupIIIPose, pickedIIIPose))
                .setLinearHeadingInterpolation(pickupIIIPose.getHeading(), pickedIIIPose.getHeading())
                .build();

        iii2shoot = follower.pathBuilder()
                .addPath(new BezierLine(pickedIIIPose, shootPose))
                .setLinearHeadingInterpolation(pickedIIIPose.getHeading(), shootPose.getHeading())
                .build();

        shoot2ii = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupIIPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupIIPose.getHeading())
                .build();

        pickii = follower.pathBuilder()
                .addPath(new BezierLine(pickupIIPose, pickedIIPose))
                .setLinearHeadingInterpolation(pickupIIPose.getHeading(), pickedIIPose.getHeading())
                .build();

        ii2shoot = follower.pathBuilder()
                .addPath(new BezierCurve(pickedIIPose, controlPoint2, shootPose))
                .setLinearHeadingInterpolation(pickedIIPose.getHeading(), shootPose.getHeading())
                .build();

        shoot2i = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupIPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupIPose.getHeading())
                .build();

        picki = follower.pathBuilder()
                .addPath(new BezierLine(pickupIPose, pickedIPose))
                .setLinearHeadingInterpolation(pickupIPose.getHeading(), pickedIPose.getHeading())
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
        double real_time;
        real_time = pathTimer.getElapsedTimeSeconds();

        double new_requerid_time;
        //new_requerid_time = real_time + 3;

        switch (pathState) {

            case 0:
                follower.followPath(startpath);
                setPathState(1);

                /*if (real_time > 7) {
                    setPathState(1);
                }*/
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                if (!follower.isBusy()) {
                    follower.followPath(shoot2iii, true);
                    //setPathState(2);
                    new_requerid_time = real_time + 3;
                    if (real_time > new_requerid_time) {
                        setPathState(2);
                    }
                }


                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pickiii, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(iii2shoot, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(shoot2ii, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pickii, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(ii2shoot, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(shoot2i, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(picki, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(i2shoot, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(11);
                }
                break;
            case 11:
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
