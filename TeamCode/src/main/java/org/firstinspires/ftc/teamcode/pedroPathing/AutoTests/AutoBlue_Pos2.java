package org.firstinspires.ftc.teamcode.pedroPathing.AutoTests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AutoBlue_Pos2", group = "Autonomous")
public class AutoBlue_Pos2 extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56.000, 9.000, Math.toRadians(90));               //Starting Pose TL:Path #1
    private final Pose startPose_CP = new Pose(64.000, 66.000);                               //Control Point (CP) for Bezier Curve TL:Path #1
    private final Pose searchPose = new Pose(54.000, 101.000, Math.toRadians(135));           //Search QR TL:Path #2
    private final Pose fst_itk_Pose = new Pose(40.000, 84.000, Math.toRadians(68));           //Position for the first intake TL:Path #3
    private final Pose fst_itk_Pose_CP = new Pose(49, 84);                                    //Control Point (CP) for Bezier Curve TL:Path #3
    private final Pose itk_Pose = new Pose(19.000, 84.000, Math.toRadians(180)); //First Intake TL:Path #4
    private final Pose scorePose = new Pose(54.000, 101.000, Math.toRadians(180)); //Position for the first shooter TL:Path #5
    private final Pose pickup1Pose = new Pose(40.000, 60.000, Math.toRadians(135)); //Position for the second intake TL:Path #6
    private final Pose pickup2Pose = new Pose(51.000, 60.000); //Control Point (CP) for Bezier Curve TL:Path #6
    private final Pose pickup3Pose = new Pose(19.000, 60.000, Math.toRadians(180)); //Second Intake TL:Path #7
    private final Pose score2Pose = new Pose(51.000, 101.000, Math.toRadians(180)); //Position for second shooter TL:Path #8
    private final Pose score23Pose = new Pose(16.000, 79.000, Math.toRadians(135)); //Parking TL:Path #9
    private final Pose scorePggose = new Pose(40.000, 77.000); //Control Point (CP) for Bezier Curve TL:Path #9
}
