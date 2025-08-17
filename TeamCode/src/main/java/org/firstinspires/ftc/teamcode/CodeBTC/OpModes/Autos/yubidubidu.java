package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.Autos;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CodeBTC.Constants;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot;

@Autonomous(name = "yahuuuuuuuuuuuuuuuuuu")
public class yubidubidu extends LinearOpMode {
    int forward=1;

    private Robot robot;
   Pose startPose=new Pose(0,0,0);
   Pose midPose=new Pose(20,20,0);
   Pose finishPose=new Pose(20,0,0);
   Path salamiiii=new Path(new BezierLine(new Point(startPose), new Point(midPose)));
   Path ehhSalamiii=new Path(new BezierLine(new Point(midPose),new Point(finishPose)));
   Path nuSalamiii=new Path(new BezierLine(new Point(finishPose), new Point(startPose) ));
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, this.startPose, true, Constants.Color.Blue, telemetry);
waitForStart();
        while(opModeIsActive()){
            robot.update();
            if (robot.isDone()) {
                if (forward==1) {
                    forward = 2;
                    robot.drive.followPath(salamiiii,true);
                } else if (forward==2) {
                    forward = 3;
                  robot.drive.followPath(ehhSalamiii,true);
                }
                else{
                    forward=1;
                            robot.drive.followPath(nuSalamiii,true);
                }
            }
            robot.drive.drawOnDashBoard();

        }
    }
}
