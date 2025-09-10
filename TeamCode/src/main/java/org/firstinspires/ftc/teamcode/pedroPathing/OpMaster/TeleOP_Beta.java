package org.firstinspires.ftc.teamcode.pedroPathing.OpMaster;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeleOP_Beta extends OpMode {
    public static Follower follower;

    @Override
    public void init() {}


    @Override
    public void start() {

        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

    }
}
