package org.firstinspires.ftc.teamcode.pedroPathing.AutoTests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OpMaster.Mecanismos;

@Autonomous()
@Disabled
public class AutoRedA2 extends OpMode {
    Mecanismos mecanism = new Mecanismos();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    public double timeStamp;

    private final Pose startPose = new Pose(32, 135.5, Math.toRadians(0)).mirror();    //PT: Start Pose of robot.
    private final Pose startCP = new Pose(45, 120).mirror();                           //CP: CONTROL POINT FOR STP-SHP.
//    private final Pose obeliskPose = new Pose(60, 132, Math.toRadians(55));       //PT: ROBOT IS SEEING THE OBELISK.
    private final Pose shootPose = new Pose(60, 132, Math.toRadians(180)).mirror();    //PT: Robot is horizontally seeing the blue goal.
    private final Pose parkPose = new Pose(15, 110, Math.toRadians(0)).mirror();       //PT: Robot tangent to blue ramp.

    private Path startpath;
    private PathChain park; //obelisk2IIIBlue

    public void buildPaths() {
        startpath = new Path(new BezierCurve(startPose, startCP, shootPose));
        startpath.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

//        obelisk2IIIBlue = follower.pathBuilder()
//                .addPath(new BezierLine(obeliskPose, pickupIIIPose))
//                .setLinearHeadingInterpolation(obeliskPose.getHeading(), pickupIIIPose.getHeading())
//                .build();

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
                setPathState(1);
                timeStamp = actualTime;
                //mecanism.shoot(0.45);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if (!follower.isBusy() && actualTime >= timeStamp + 1)
                    mecanism.intake(1.0);
                if (!follower.isBusy() && actualTime >= timeStamp + 5) {
                    //mecanism.shoot(0);
                    follower.followPath(park,true);
                    setPathState(2);
                }
                break;
            case 2:
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