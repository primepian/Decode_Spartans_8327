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

@Autonomous(name = "AutoBlue_Pos2", group = "Autonomous")
@Disabled
public class AutoBlue_Pos2 extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56.000, 9.000, Math.toRadians(90));               //Starting and first Position for shooting Pose TL:Path #1
    private final Pose startPose_CP = new Pose(64.000, 66.000);                               //Control Point (CP) for Bezier Curve           TL:Path #1
    private final Pose searchPose = new Pose(54.000, 101.000, Math.toRadians(135));           //Search QR                                     TL:Path #2
    private final Pose fst_itk_Pose = new Pose(40.000, 84.000, Math.toRadians(68));           //Position for the first intake                 TL:Path #3
    private final Pose fst_itk_Pose_CP = new Pose(49, 84);                                    //Control Point (CP) for Bezier Curve           TL:Path #3
    private final Pose fst_itk = new Pose(19.000, 84.000, Math.toRadians(180));               //First Intake                                  TL:Path #4
    private final Pose snd_shooter_Pose = new Pose(54.000, 101.000, Math.toRadians(180));     //Position for the second shooter               TL:Path #5
    private final Pose snd_itk_Pose = new Pose(40.000, 60.000, Math.toRadians(135));          //Position for the second intake                TL:Path #6
    private final Pose snd_itk_Pose_CP = new Pose(51.000, 60.000);                            //Control Point (CP) for Bezier Curve           TL:Path #6
    private final Pose snd_itk = new Pose(19.000, 60.000, Math.toRadians(180));               //Second Intake                                 TL:Path #7
    private final Pose trd_shooter_Pose = new Pose(51.000, 101.000, Math.toRadians(180));     //Position for second shooter                   TL:Path #8
    private final Pose parking = new Pose(16.000, 79.000, Math.toRadians(135));               //Parking                                       TL:Path #9
    private final Pose parking_CP = new Pose(40.000, 77.000);                                 //Control Point (CP) for Bezier Curve           TL:Path #9

    private Path start_path;
    private PathChain fst_path, snd_path, trd_path, fth_path, fvth_path, sxth_path, svnth_path, egth_path, nnth_path;

    public void buildPaths() {

        start_path = new Path(new BezierCurve(startPose, startPose_CP, searchPose));
        start_path.setLinearHeadingInterpolation(startPose.getHeading(), searchPose.getHeading());

        fst_path = follower.pathBuilder()
                .addPath(new BezierLine(searchPose, fst_itk_Pose))
                .setLinearHeadingInterpolation(searchPose.getHeading(), fst_itk_Pose.getHeading())
                .build();

        snd_path = follower.pathBuilder()
                .addPath(new BezierCurve(fst_itk_Pose, fst_itk_Pose_CP, fst_itk))
                .setLinearHeadingInterpolation(fst_itk_Pose.getHeading(), fst_itk.getHeading())
                .build();

        trd_path = follower.pathBuilder()
                .addPath(new BezierLine(fst_itk, snd_shooter_Pose))
                .setLinearHeadingInterpolation(fst_itk.getHeading(), snd_shooter_Pose.getHeading())
                .build();

        fth_path = follower.pathBuilder()
                .addPath(new BezierLine(snd_shooter_Pose, snd_itk_Pose))
                .setLinearHeadingInterpolation(snd_shooter_Pose.getHeading(), snd_itk_Pose.getHeading())
                .build();

        fvth_path = follower.pathBuilder()
                .addPath(new BezierCurve(snd_itk_Pose, snd_itk_Pose_CP, snd_itk))
                .setLinearHeadingInterpolation(snd_itk_Pose.getHeading(), snd_itk.getHeading())
                .build();

        sxth_path = follower.pathBuilder()
                .addPath(new BezierLine(snd_itk, trd_shooter_Pose))
                .setLinearHeadingInterpolation(snd_itk.getHeading(), trd_shooter_Pose.getHeading())
                .build();

        svnth_path = follower.pathBuilder()
                .addPath(new BezierLine(trd_shooter_Pose, parking))
                .setLinearHeadingInterpolation(trd_shooter_Pose.getHeading(), parking.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(start_path);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(fst_path,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(snd_path,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(trd_path,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(fth_path,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(fvth_path,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(sxth_path, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(svnth_path, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
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
