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
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.ArmIntake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.LinearSlides;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.WristIntake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot;

import java.util.ArrayList;

@Autonomous(name = "bliah b lah")
public class AutoSpecBlue extends LinearOpMode{

    private Robot robot;
    public Pose startPose = new Pose(7.5, 64,0);
    private final Pose scorePose = new Pose(35, 68,0);
    private final Pose collectPose = new Pose(10, 37.5,0);
    private ElapsedTime timer = new ElapsedTime();
    Pose[] waitCollectSpike = {new Pose(30 , 40 , -1.1) , new Pose(35, 30 , -1.2) , new Pose(32 , 20 , -1)};
    Pose[] collectSpike = {new Pose(33 , 37 , -1.1) , new Pose(38, 27 , -1.2) , new Pose(35 , 17 , -1)};
    Pose[] spitPose = {new Pose(38 , 32 , -2.3) , new Pose(38, 27 , -2.3) , new Pose(35 , 17 , -2.3)};
    double[] waitExtendPoses ={0.13 , 0.2 , 0.24};
    double[] extendPoses ={0.18 , 0.25 , 0.3};
    private final Pose controlPointPose = new Pose(11.5, 32, 0);

    BezierCurve goToSpike = new BezierCurve(scorePose ,new Pose(15, 53), waitCollectSpike[0]);
    BezierLine scorePreload = new BezierLine(startPose, scorePose);
    BezierLine[] goCollectAndSpit = {new BezierLine(waitCollectSpike[0] , collectSpike[0]) ,
            new BezierLine(collectSpike[0] , spitPose[0]),
            new BezierLine(spitPose[0], waitCollectSpike[1]),
            new BezierLine(waitCollectSpike[1] , collectSpike[1]) ,
            new BezierLine(collectSpike[1] , spitPose[1]),
            new BezierLine(spitPose[1], waitCollectSpike[2]),
            new BezierLine(waitCollectSpike[2] , collectSpike[2]) ,
            new BezierLine(collectSpike[2] , spitPose[2]),
            new BezierLine(spitPose[2], collectPose)};

    Path[] pathChain = new Path[20];

    private PathChain fullAutoPath;
    private int CurrentPathIndex=0;
    private void CheckIfCollected(){
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, this.startPose, true, Constants.Color.Blue, telemetry);
        pathChain[0] = new Path(scorePreload);
        pathChain[0].setConstantHeadingInterpolation(0);
        pathChain[1] = new Path(goToSpike);
        pathChain[1].setLinearHeadingInterpolation(scorePose.getHeading() , waitCollectSpike[0].getHeading());
        pathChain[2] = new Path(goCollectAndSpit[0]);
        pathChain[2].setConstantHeadingInterpolation(collectSpike[0].getHeading());
        pathChain[3] = new Path(goCollectAndSpit[1]);
        pathChain[3].setLinearHeadingInterpolation(collectSpike[0].getHeading() , spitPose[0].getHeading());
        pathChain[4] = new Path(goCollectAndSpit[2]);
        pathChain[4].setLinearHeadingInterpolation(spitPose[0].getHeading() , waitCollectSpike[1].getHeading());
        pathChain[5] = new Path(goCollectAndSpit[3]);
        pathChain[5].setConstantHeadingInterpolation(collectSpike[1].getHeading());
        pathChain[6] = new Path(goCollectAndSpit[4]);
        pathChain[6].setLinearHeadingInterpolation(collectSpike[1].getHeading() , spitPose[1].getHeading());
        int state = 1;
        waitForStart();

        if (isStopRequested()) return;
        while(opModeIsActive()){
            switch (state){
                case 1:
                    robot.drive.followPath(pathChain[0]);
                    state++;
                    break;
                case 2:
                    if(robot.isDone()){
                        robot.drive.followPath(pathChain[1]);
                        timer.reset();
                        state++;
                    }
                    break;
                case 3:
                    if(timer.milliseconds() >= 400){
                        robot.armIntake.setState(ArmIntake.States.Collect);
                        robot.wristIntake.setState(WristIntake.States.Collect);
                        robot.activeIntake.setState(ActiveIntake.States.Collect);
                        robot.slides.setTarget(extendPoses[0]);
                        state++;
                    }
                    break;
                case 4:
                    if(robot.isDone()){
                        robot.drive.followPath(pathChain[2]);
                        timer.reset();
                        state++;
                    }
                    break;
                case 5:
                    if(robot.activeIntake.haveCollected() || timer.milliseconds() > 650){
                        robot.drive.followPath(pathChain[3]);
                        robot.armIntake.setState(ArmIntake.States.WaitSpecificOne);
                        robot.wristIntake.setState(WristIntake.States.WaitCollectSpecificOne);
                        state++;
                    }
                    break;
                case 6:
                    if(robot.isDone()){
                        robot.activeIntake.setState(ActiveIntake.States.Spit);
                        state++;
                    }
                    break;
            }

            robot.update();
        }
    }
}