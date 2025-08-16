package org.firstinspires.ftc.teamcode.CodeBTC.PedroStuff.TunersTests.PID;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.slidesRetractedPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CodeBTC.Constants;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.LinearSlides;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@TeleOp(name = "Test Wow", group = "PIDF Tuning")
public class TestWOW extends LinearOpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;
    LinearSlides slides;

    private Path forwards;
    private Path backwards;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap, Constants.FConstants.class, Constants.LConstants.class);

        forwards = new Path(new BezierLine(new Pose(0, 0, Math.toRadians(0)), new Pose(35, 10, Math.toRadians(90))));
        forwards.setLinearHeadingInterpolation(0,  Math.toRadians(90));
        backwards = new Path(new BezierLine(new Pose(35, 10, Math.toRadians(90)), new Pose(0, 0, Math.toRadians(0))));
        backwards.setLinearHeadingInterpolation(Math.toRadians(90), 0);
        slides = new LinearSlides(hardwareMap, slidesRetractedPose, true);

        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            if (!follower.isBusy()) {
                if (forward) {
                    forward = false;
                    follower.followPath(backwards);
                } else {
                    forward = true;
                    follower.followPath(forwards);
                }
            }

            telemetryA.addData("going forward", forward);
            telemetryA.addData("is busy", follower.isBusy());
            follower.telemetryDebug(telemetryA);
        }
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     * */

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
}
