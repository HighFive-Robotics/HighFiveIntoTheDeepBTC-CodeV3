package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.Autos;

import com.acmerobotics.dashboard.config.Config;
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
    Pose[] waitCollectSpike = {new Pose(23.5 , 45 , -0.9) , new Pose(23.5, 40 , -1.1) , new Pose(23.5 , 35 , -1)};
    Pose[] collectSpike = {new Pose(27 , 48 , -0.9) , new Pose(27, 43 , -1.1) , new Pose(27 , 30 , -1)};
    Pose[] spitPose = {new Pose(23.5 , 35 , -2.3) , new Pose(38, 27 , -2.3) , new Pose(35 , 17 , -2.3)};
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
        pathChain[7] = new Path(goCollectAndSpit[5]);
        pathChain[7].setLinearHeadingInterpolation(spitPose[1].getHeading() , waitCollectSpike[2].getHeading());
        pathChain[8] = new Path(goCollectAndSpit[6]);
        pathChain[8].setConstantHeadingInterpolation(collectSpike[2].getHeading());
        pathChain[9] = new Path(goCollectAndSpit[2]);
        pathChain[9].setLinearHeadingInterpolation(collectSpike[2].getHeading() , spitPose[2].getHeading());
        int state = 1;
        waitForStart();

        if (isStopRequested()) return;
        while(opModeIsActive()){
            switch (state){
                case 1:
                    robot.drive.followPath(pathChain[0], true);
                    robot.setAction(Robot.Actions.GoToScoreSpecimen);
                    state++;
                    break;
                case 2:
                    if(robot.isDone()){
                        robot.drive.followPath(pathChain[1], true);
                        robot.setAction(Robot.Actions.GoToCollectSpecimen);
                        timer.reset();
                        state++;
                    }
                    break;
                case 3:
                    if(timer.milliseconds() >= 400){
                        robot.armIntake.setState(ArmIntake.States.Collect);
                        robot.wristIntake.setState(WristIntake.States.Collect);
                        robot.activeIntake.setState(ActiveIntake.States.Collect);
                        robot.slides.setTarget(waitExtendPoses[0]);
                        state++;
                    }
                    break;
                case 4:
                    if(robot.isDone()){
                        robot.drive.followPath(pathChain[2], true);
                        robot.slides.setTarget(extendPoses[0]);
                        timer.reset();
                        state++;
                    }
                    break;
                case 5:
                    if(robot.activeIntake.haveCollected() || timer.milliseconds() > 650){
                        robot.drive.followPath(pathChain[3], true);
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
                case 7:
                    if(robot.isDone()){
                        robot.drive.followPath(pathChain[4], true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 8:
                    if(timer.milliseconds() >= 400){
                        robot.armIntake.setState(ArmIntake.States.Collect);
                        robot.wristIntake.setState(WristIntake.States.Collect);
                        robot.activeIntake.setState(ActiveIntake.States.Collect);
                        robot.slides.setTarget(waitExtendPoses[1]);
                        state++;
                    }
                    break;
                case 9:
                    if(robot.isDone()){
                        robot.drive.followPath(pathChain[5], true);
                        robot.slides.setTarget(extendPoses[1]);
                        timer.reset();
                        state++;
                    }
                    break;
                case 10:
                    if(robot.activeIntake.haveCollected() || timer.milliseconds() > 650){
                        robot.drive.followPath(pathChain[6], true);
                        robot.armIntake.setState(ArmIntake.States.WaitSpecificOne);
                        robot.wristIntake.setState(WristIntake.States.WaitCollectSpecificOne);
                        state++;
                    }
                    break;
                case 11:
                    if(robot.isDone()){
                        robot.activeIntake.setState(ActiveIntake.States.Spit);
                        state++;
                    }
                    break;
                case 12:
                    if(robot.isDone()){
                        robot.drive.followPath(pathChain[7], true);
                        timer.reset();
                        state++;
                    }
                    break;
                case 13:
                    if(timer.milliseconds() >= 400){
                        robot.armIntake.setState(ArmIntake.States.Collect);
                        robot.wristIntake.setState(WristIntake.States.Collect);
                        robot.activeIntake.setState(ActiveIntake.States.Collect);
                        robot.slides.setTarget(waitExtendPoses[2]);
                        state++;
                    }
                    break;
                case 14:
                    if(robot.isDone()){
                        robot.drive.followPath(pathChain[8], true);
                        robot.slides.setTarget(extendPoses[2]);
                        timer.reset();
                        state++;
                    }
                    break;
                case 15:
                    if(robot.activeIntake.haveCollected() || timer.milliseconds() > 650){
                        robot.drive.followPath(pathChain[9], true);
                        robot.armIntake.setState(ArmIntake.States.WaitSpecificOne);
                        robot.wristIntake.setState(WristIntake.States.WaitCollectSpecificOne);
                        state++;
                    }
                    break;
                case 16:
                    if(robot.isDone()){
                        robot.activeIntake.setState(ActiveIntake.States.Spit);
                        state++;
                    }
                    break;
            }

            robot.drive.drawOnDashBoard();
            robot.update();
        }
    }
}