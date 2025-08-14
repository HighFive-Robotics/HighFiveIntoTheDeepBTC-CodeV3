package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.Autos;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CodeBTC.Constants;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot;

@Autonomous
public class AutoTest extends LinearOpMode {

    private Robot robot;
    private final ElapsedTime timer = new ElapsedTime();
    private int pathState = 1;
    int cycles = 0;

    public Pose startPose = new Pose(7.5, 64, Math.toRadians(0));
    private final Pose scorePose = new Pose(32, 78, Math.toRadians(0));
    private final Pose collectPose = new Pose(6, 30, Math.toRadians(0));
    private final Pose controlPointPose = new Pose(11.5, 32, Math.toRadians(0));
    private final Pose preparePushingPose = new Pose(60, 34, Math.toRadians(0));
    private final Pose pushSamplePose1 = new Pose(60, 24, Math.toRadians(0));
    private final Pose pushSamplePose2 = new Pose(60, 14, Math.toRadians(0));
    private final Pose pushSamplePose3 = new Pose(60, 6, Math.toRadians(0));
    private final Pose observationZonePose1 = new Pose(25, 24, Math.toRadians(0));
    private final Pose observationZonePose2 = new Pose(25, 14, Math.toRadians(0));
    private final Pose observationZonePose3 = new Pose(25, 6, Math.toRadians(0));

    private PathChain scorePreload, scoreSpecimen, crazyBezier, goSpikeMark1, goSpikeMark2, goSpikeMark3, pushSample1, pushSample2, pushSample3, goToCollectSpecimen, collectSpecimen;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, this.startPose, true, Constants.Color.Blue, telemetry);

        scorePreload = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        crazyBezier = robot.drive.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(controlPointPose), new Point(preparePushingPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preparePushingPose.getHeading())
                .build();
        goSpikeMark1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(preparePushingPose), new Point(pushSamplePose1)))
                .setLinearHeadingInterpolation(preparePushingPose.getHeading(), pushSamplePose1.getHeading())
                .build();
        pushSample1 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(pushSamplePose1), new Point(observationZonePose1)))
                .setLinearHeadingInterpolation(pushSamplePose1.getHeading(), observationZonePose1.getHeading())
                .build();
        goSpikeMark2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(observationZonePose1), new Point(pushSamplePose2)))
                .setLinearHeadingInterpolation(observationZonePose1.getHeading(), pushSamplePose2.getHeading())
                .build();
        pushSample2 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(pushSamplePose2), new Point(observationZonePose2)))
                .setLinearHeadingInterpolation(pushSamplePose2.getHeading(), observationZonePose2.getHeading())
                .build();
        goSpikeMark3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(observationZonePose2), new Point(pushSamplePose3)))
                .setLinearHeadingInterpolation(observationZonePose2.getHeading(), pushSamplePose3.getHeading())
                .build();
        pushSample3 = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(pushSamplePose3), new Point(observationZonePose3)))
                .setLinearHeadingInterpolation(pushSamplePose3.getHeading(), observationZonePose3.getHeading())
                .build();
        goToCollectSpecimen = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(observationZonePose3), new Point(collectPose)))
                .setLinearHeadingInterpolation(observationZonePose3.getHeading(), collectPose.getHeading())
                .build();
        scoreSpecimen = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(collectPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(collectPose.getHeading(), scorePose.getHeading())
                .build();
        collectSpecimen = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(collectPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), collectPose.getHeading())
                .build();

        waitForStart();

        while(opModeIsActive()){
            switch (pathState){
                case 1:
                    robot.drive.followPath(scorePreload);
                    robot.setAction(Robot.Actions.GoToScoreSpecimen);
                    pathState = 2;
                    break;
                case 2:
                    if(robot.isDone()){
                        robot.drive.followPath(crazyBezier);
                        robot.setAction(Robot.Actions.GoToCollectSpecimen);
                        pathState = 3;
                    }
                    break;
                case 3:
                    if(robot.isDone()){
                        robot.drive.followPath(goSpikeMark1, true);
                        pathState = 4;
                    }
                    break;
                case 4:
                    if(robot.isDone()){
                        robot.drive.followPath(pushSample1, true);
                        pathState = 5;
                    }
                    break;
                case 5:
                    if(robot.isDone()){
                        robot.drive.followPath(goSpikeMark2, true);
                        pathState = 7;
                    }
                    break;
                case 7:
                    if(robot.isDone()){
                        robot.drive.followPath(pushSample2, true);
                        pathState = 8;
                    }
                    break;
                case 8:
                    if(robot.isDone()){
                        robot.drive.followPath(goSpikeMark2, true);
                        pathState = 9;
                    }
                    break;
                case 9:
                    if(robot.isDone()){
                        robot.drive.followPath(pushSample2, true);
                        pathState = 10;
                    }
                    break;
                case 10:
                    if(robot.isDone()){
                        robot.drive.followPath(goSpikeMark3, true);
                        pathState = 11;
                    }
                    break;
                case 11:
                    if(robot.isDone()){
                        robot.drive.followPath(pushSample3, true);
                        pathState = 12;
                    }
                    break;
                case 12:
                    if(robot.isDone()){
                        robot.drive.followPath(goToCollectSpecimen, true);
                        pathState = 13;
                    }
                    break;
                case 13:
                    if(robot.isDone()){
                        robot.setAction(Robot.Actions.GoToScoreSpecimen);
                        timer.reset();
                        cycles++;
                        pathState = 14;
                    }
                    break;
                case 14:
                    if(timer.milliseconds() >= 250){
                        robot.drive.followPath(scoreSpecimen, true);
                        pathState = 15;
                    }
                    break;
                case 15:
                    if(robot.isDone()){
                        robot.drive.followPath(collectSpecimen, true);
                        pathState = 16;
                    }
                    break;
                case 16:
                    if(cycles < 4){
                        pathState = 13;
                    } else {
                        pathState++;
                    }
                    break;
            }
            robot.update();
            telemetry.addData("State", pathState);
            telemetry.addData("Pose", robot.drive.getPose());
            telemetry.addData("blahblah", robot.drive.poseUpdater.getLocalizer().getForwardMultiplier());
            telemetry.update();
        }
    }
}
