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

@Autonomous(group = "Emergencia")
public class RED_EMERGENCIA extends OpMode{
    Mecanismos mecanism = new Mecanismos();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startingPose = new Pose(55.000, 8.000, Math.toRadians(90)).mirror();            //TL:Path #1
    private final Pose second = new Pose(62, 25, Math.toRadians(114)).mirror();           //TL:Path #2 TODO: Shoot fixme:57.000, 105.000, Math.toRadians(145)
    private final Pose tird = new Pose(35, 8, Math.toRadians(90)).mirror();           //TL:Path #2 TODO: Shoot fixme:57.000, 105.000, Math.toRadians(145)


    private Path start_path, second_path;

    public void buildPaths() {

        start_path = new Path(new BezierLine(startingPose, second));
        start_path.setLinearHeadingInterpolation(startingPose.getHeading(), second.getHeading());

        second_path = new Path(new BezierLine(second, tird));
        second_path.setLinearHeadingInterpolation(second.getHeading(), tird.getHeading());

    }

    public void autonomousPathUpdate() {
        double actual_time = pathTimer.getElapsedTimeSeconds();

        switch (pathState) {
            case 0: //start to obelisk
                follower.followPath(start_path);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    mecanism.shootMax();
                    setPathState(2);
                }
                break;
            case 2:
                if ((!follower.isBusy()) && !mecanism.isShooting) {
                    follower.followPath(second_path);
                    setPathState(3);
                }
                break;
            case 3:
                if ((!follower.isBusy())) {
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
        mecanism.shootingandIntakeMax(telemetry);

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