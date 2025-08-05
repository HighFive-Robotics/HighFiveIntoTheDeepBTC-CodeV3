package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.TeleOps;



import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Others.Climb.States.Climbing;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Others.Climb.States.ResetLeft;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Others.Climb.States.ResetRight;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Others.Climb.States.ReverseClimb;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Others.Climb.States.Waiting;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.IntakeGoToTransfer;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.OuttakeGoToTransferSample;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.SpecimenWithIntakeUp;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.StartCollecting;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.StartCollectingSpecific;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.StartCollectingWithExtension;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.StartCollectingWithWait;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.TransferSample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CodeBTC.Constants;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.LinearSlides;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake.Claw;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake.Lift;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake.LinkageOuttake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot;

@TeleOp(name = "🔵TeleOpBlue🔵")
public class TeleOpBlue extends LinearOpMode {
    ElapsedTime timerA1 = new ElapsedTime(),  timerB1= new ElapsedTime(),timerX1 = new ElapsedTime(), timerY1 = new ElapsedTime(), timerA2 = new ElapsedTime(), timerB2 = new ElapsedTime(), timerX2 = new ElapsedTime(), timerY2 = new ElapsedTime();
    ElapsedTime timerLeftSitckButton2 = new ElapsedTime(), timerRightSitckButton2 = new ElapsedTime(), timerLeftBumper2 = new ElapsedTime(), timerRightBumper2 = new ElapsedTime();
    ElapsedTime timerLeftBumper1 = new ElapsedTime(), timerRightBumper1 = new ElapsedTime(), timerTouchPad = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime(), timer2 = new ElapsedTime();
    Robot robot;
    Pose startPose = new Pose(0,0,0);
    boolean intakeShouldGoToTransferAutomatically = false;

    public enum CollectType{
        Specific,
        Normal,
        WithExtension,
        WithWait
    }

    CollectType collectType = CollectType.Normal;

    @Override
    public void runOpMode() throws InterruptedException {

        if(Constants.Globals.isSampleAuto){
            startPose = new Pose(0,0,90);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, startPose, false, Constants.Color.Blue, telemetry);

        telemetry.addLine("OK");

        telemetry.update();
        robot.drive.startTeleopDrive();
        waitForStart();
        robot.update();
        while (opModeIsActive()) {
            //Driver 1 Controls
            if (gamepad1.ps) {
                robot.drive.setStartingPose(robot.drive.getPose());
                Constants.Globals.isSampleAuto = false;
            }
            robot.drive.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);

            if (gamepad1.a && timerA1.milliseconds() >= 250) {
                robot.setAction(SpecimenWithIntakeUp);
                timerA1.reset();
            }


            if (gamepad1.b && timerB1.milliseconds() >= 250) {
                robot.setAction(OuttakeGoToTransferSample);
                robot.setAction(StartCollectingSpecific);
                intakeShouldGoToTransferAutomatically = true;
                timerB1.reset();
            }

            if (gamepad1.left_bumper && timerLeftBumper1.milliseconds() >= 250) {
                robot.claw.setState(Claw.States.Open);
                timerLeftBumper1.reset();
            }

            if (gamepad1.right_bumper && timerRightBumper1.milliseconds() >= 250) {
                gamepad2.rumble(50);
                timerRightBumper1.reset();
            }

            robot.climb.setState(Waiting);
            if (gamepad1.right_trigger >= 0.7) {
                robot.climb.setState(Climbing);
            }
            if (gamepad1.left_trigger >= 0.7) {
                robot.climb.setState(ReverseClimb);
            }
            if (gamepad1.dpad_left) {
                robot.climb.setState(ResetLeft);
            }
            if (gamepad1.dpad_right) {
                robot.climb.setState(ResetRight);
            }

            if (gamepad1.left_stick_button && gamepad1.right_bumper && timer1.milliseconds() >= 250) {
                robot.armOuttake.setTarget(robot.armOuttake.getTarget() - 0.02);
                timer1.reset();
            }
            if (gamepad1.right_stick_button && gamepad1.right_bumper && timer2.milliseconds() >= 250) {
                robot.armOuttake.setTarget(robot.armOuttake.getTarget() + 0.02);
                timer2.reset();
            }
            //Driver 2 Controls

            if (gamepad2.a && timerA2.milliseconds() >= 250) {
                robot.setAction(TransferSample);
                timerA2.reset();
            }
            if (gamepad2.x && timerX2.milliseconds() >= 250) {
                robot.setAction(IntakeGoToTransfer);
                intakeShouldGoToTransferAutomatically = false;
                timerX2.reset();
            }
            if (gamepad2.b && timerB2.milliseconds() >= 250) {
                robot.setAction(OuttakeGoToTransferSample);
                timerB2.reset();
            }

            if (gamepad2.left_bumper && timerLeftBumper2.milliseconds() >= 250) {
                robot.claw.setState(Claw.States.Open);
                timerLeftBumper2.reset();
            }

            if (gamepad2.right_bumper && timerRightBumper2.milliseconds() >= 250) {
                robot.claw.setState(Claw.States.Closed);
                timerRightBumper2.reset();
            }

            if (gamepad2.left_stick_button && timerLeftSitckButton2.milliseconds() >= 250) {
                if (gamepad2.left_stick_x >= 0.8) {
                    collectType = CollectType.Specific;
                    timerLeftSitckButton2.reset();
                }
                if (gamepad2.left_stick_x <= -0.8) {
                    collectType = CollectType.Normal;
                    timerLeftSitckButton2.reset();
                }
                if (-gamepad2.left_stick_y >= 0.8) {
                    collectType = CollectType.WithExtension;
                    timerLeftSitckButton2.reset();
                }
                if (-gamepad2.left_stick_y <= -0.8) {
                    collectType = CollectType.WithWait;
                    timerLeftSitckButton2.reset();
                }
            }

            switch (collectType) {
                case Specific:
                    if (gamepad2.right_stick_button && timerRightSitckButton2.milliseconds() >= 250) {
                        robot.setAction(StartCollectingSpecific);
                        intakeShouldGoToTransferAutomatically = !robot.activeIntake.haveCollected();
                        timerRightSitckButton2.reset();
                    }
                    break;
                case Normal:
                    if (gamepad2.right_stick_button && timerRightSitckButton2.milliseconds() >= 250) {
                        robot.setAction(StartCollecting);
                        intakeShouldGoToTransferAutomatically = !robot.activeIntake.haveCollected();
                        timerRightSitckButton2.reset();
                    }
                    break;
                case WithExtension:
                    if (gamepad2.right_stick_button && timerRightSitckButton2.milliseconds() >= 250) {
                        robot.setAction(StartCollectingWithExtension);
                        intakeShouldGoToTransferAutomatically = !robot.activeIntake.haveCollected();
                        timerRightSitckButton2.reset();
                    }
                    break;
                case WithWait:
                    if (gamepad2.right_stick_button && timerRightSitckButton2.milliseconds() >= 250) {
                        robot.setAction(StartCollectingWithWait);
                        intakeShouldGoToTransferAutomatically = !robot.activeIntake.haveCollected();
                        timerRightSitckButton2.reset();
                    }
                    break;
            }

            if (gamepad2.right_trigger >= 0.7) {
                robot.activeIntake.setState(ActiveIntake.States.Spit, false);
            }
            if (gamepad2.left_trigger >= 0.7) {
                robot.activeIntake.setState(ActiveIntake.States.Collect);
            }
            if (gamepad2.right_trigger >= 0.7 && gamepad2.left_trigger >= 0.7) {
                robot.activeIntake.setState(ActiveIntake.States.Wait);
            }

            if (gamepad2.dpad_left) {
                robot.slides.setState(LinearSlides.States.Retracted);
            }

            if (gamepad2.dpad_up) {
                robot.lift.setTarget(robot.lift.getTarget() + 10);
            }

            if (gamepad2.dpad_down) {
                robot.lift.setTarget(robot.lift.getTarget() - 10);
            }

            if (gamepad2.dpad_right) {
                robot.lift.setState(Lift.States.LowBasket);
                robot.linkageOuttake.setState(LinkageOuttake.States.Extended);
            }

            if (gamepad2.right_stick_y >= 0.6 || gamepad2.right_stick_y <= -0.6) {
                robot.slides.setTargetJoyStick(-gamepad2.right_stick_y);
            }

            if(robot.activeIntake.getState() == ActiveIntake.States.Spit){
                gamepad2.rumble(250);
            }

            if (gamepad2.touchpad_finger_1 && gamepad2.touchpad_finger_2 && timerTouchPad.milliseconds() >= 250) {
                gamepad1.rumble(50);
                timerTouchPad.reset();
            }

            //Update
            if (intakeShouldGoToTransferAutomatically && robot.activeIntake.haveCollected()) {
                intakeShouldGoToTransferAutomatically = false;
                robot.setAction(IntakeGoToTransfer);
                gamepad1.rumble(250);
            }

            robot.drive.update();
            robot.update();
            telemetry.addData("Target Lift", robot.lift.getTarget());
            telemetry.addData("Lift Pose", robot.lift.getCurrentPosition());
            telemetry.addData("Arm At Target: ", robot.armOuttake.atTarget());
            telemetry.addData("Linkage Outtake At Target: ", robot.linkageOuttake.atTarget());
            telemetry.addData("Wrist Outtake At Target: ", robot.wristOuttake.atTarget());
            telemetry.addData("Arm Intake At Target: ", robot.armIntake.atTarget());
            telemetry.addData("Wrist Intake At Target: ", robot.wristIntake.atTarget());
            telemetry.addData("Linear Slides At Target: ", robot.slides.atTarget());
            telemetry.addData("Sensor distance: ", robot.activeIntake.intakeSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}