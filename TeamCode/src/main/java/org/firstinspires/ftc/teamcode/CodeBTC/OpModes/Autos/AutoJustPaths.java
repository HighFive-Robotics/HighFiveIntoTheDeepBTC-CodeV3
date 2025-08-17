package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.Autos;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CodeBTC.Constants;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.LinearSlides;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot;

@Autonomous
public class AutoJustPaths extends LinearOpMode {

    private Robot robot;
    public Pose startPose = new Pose(7.5, 64, Math.toRadians(0));

    private final Pose scorePose = new Pose(34.7, 72, Math.toRadians(0));
    private final Pose collectPose = new Pose(6, 30, Math.toRadians(0));
    private final Pose controlPointPose = new Pose(11.5, 32, Math.toRadians(0));
    private final Pose preparePushingPose = new Pose(60, 34, Math.toRadians(0));
    private final Pose pushSamplePose1 = new Pose(60, 24, Math.toRadians(0));
    private final Pose pushSamplePose2 = new Pose(60, 14, Math.toRadians(0));
    private final Pose pushSamplePose3 = new Pose(60, 11, Math.toRadians(0));
    private final Pose observationZonePose1 = new Pose(25, 24, Math.toRadians(0));
    private final Pose observationZonePose2 = new Pose(25, 14, Math.toRadians(0));
    private final Pose observationZonePose3 = new Pose(25, 11, Math.toRadians(0));

    private PathChain fullAutoPath;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, this.startPose, true, Constants.Color.Blue, telemetry);

        fullAutoPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())


                .addPath(new BezierCurve(new Point(scorePose), new Point(controlPointPose), new Point(preparePushingPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), preparePushingPose.getHeading())


                .addPath(new BezierLine(new Point(preparePushingPose), new Point(pushSamplePose1)))
                .setZeroPowerAccelerationMultiplier(1)
                .setLinearHeadingInterpolation(preparePushingPose.getHeading(), pushSamplePose1.getHeading())

                .addPath(new BezierLine(new Point(pushSamplePose1), new Point(observationZonePose1)))
                    .setLinearHeadingInterpolation(pushSamplePose1.getHeading(), observationZonePose1.getHeading())
                    .setReversed(true)
                .addParametricCallback(0,() -> robot.slides.setState(LinearSlides.States.Extended))

                .addPath(new BezierLine(new Point(observationZonePose1), new Point(pushSamplePose2)))
                .setLinearHeadingInterpolation(observationZonePose1.getHeading(), pushSamplePose2.getHeading())
                .addParametricCallback(0,() -> robot.slides.setState(LinearSlides.States.Retracted))

                .addPath(new BezierLine(new Point(pushSamplePose2), new Point(observationZonePose2)))
                    .setLinearHeadingInterpolation(pushSamplePose2.getHeading(), observationZonePose2.getHeading())
                    .setReversed(true)
                .addParametricCallback(0,() -> robot.slides.setState(LinearSlides.States.Extended))


                .addPath(new BezierLine(new Point(observationZonePose2), new Point(pushSamplePose3)))
                .setLinearHeadingInterpolation(observationZonePose2.getHeading(), pushSamplePose3.getHeading())
                .addParametricCallback(0,() -> robot.slides.setState(LinearSlides.States.Retracted))

                .addPath(new BezierLine(new Point(pushSamplePose3), new Point(observationZonePose3)))
                    .setLinearHeadingInterpolation(pushSamplePose3.getHeading(), observationZonePose3.getHeading())
                    .setReversed(true)
                .addParametricCallback(0,() -> robot.slides.setState(LinearSlides.States.Extended))


                .addPath(new BezierLine(new Point(observationZonePose3), new Point(collectPose)))
                .setLinearHeadingInterpolation(observationZonePose3.getHeading(), collectPose.getHeading())
                .addParametricCallback(0,() -> robot.slides.setState(LinearSlides.States.Retracted))

                .addPath(new BezierLine(new Point(collectPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(collectPose.getHeading(), scorePose.getHeading())

                .addPath(new BezierLine(new Point(scorePose), new Point(collectPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), collectPose.getHeading())

                .addPath(new BezierLine(new Point(collectPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(collectPose.getHeading(), scorePose.getHeading())

                .addPath(new BezierLine(new Point(scorePose), new Point(collectPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), collectPose.getHeading())

                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followPath(fullAutoPath, true);

        while(opModeIsActive()){
            robot.update();
        }
    }
}