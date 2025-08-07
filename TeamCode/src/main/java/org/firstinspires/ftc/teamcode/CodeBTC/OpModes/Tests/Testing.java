package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.Tests;

import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.IntakeGoToTransfer;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.IntakeGoToTransferSpecimen;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.OuttakeGoToTransferSample;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.SpecimenWithIntakeDown;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.SpecimenWithIntakeUp;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.StartCollecting;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.StartCollectingSpecific;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.StartCollectingWithExtension;
import static org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot.Actions.StartCollectingWithWait;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CodeBTC.Constants;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake.ArmOuttake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Robot;

@TeleOp(name = "️️️️⚔️TESTING ONLY⚔️❣️")
public class Testing extends LinearOpMode {

    ElapsedTime time1 = new ElapsedTime(), time2 = new ElapsedTime(),time3=new ElapsedTime(),time4=new ElapsedTime(),time5=new ElapsedTime();
    Robot robot;
    Constants.Color allianceColor;
    Pose startPose;
    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose(0,0,0);
        boolean isAuto = false;
        allianceColor= Constants.Color.Blue;
        robot = new Robot(hardwareMap,startPose,isAuto,allianceColor,telemetry);
        robot.drive.startTeleopDrive();
        while(opModeInInit())
        {


        }
        waitForStart();

        //gamepad2.setLedColor(132, 88, 164 , 999999999);
        while (opModeIsActive()){

            if(gamepad1.x && time1.milliseconds() >= 450){
                robot.setAction(Robot.Actions.GoToCollectSpecimen);
                time1.reset();
            }
            if(gamepad1.a && time2.milliseconds() >= 450){
                robot.setAction(Robot.Actions.GoToScoreSpecimen);
                time2.reset();
            }
            if(gamepad1.x && time2.milliseconds() >= 450){
                robot.armOuttake.setState(ArmOuttake.States.ScoreSpecimen, 600);
                time2.reset();
            }
            if(gamepad1.dpad_up&&time3.milliseconds()>=250){
                robot.setAction(SpecimenWithIntakeUp);
                time3.reset();
            }
            if(gamepad1.dpad_down&&time4.milliseconds()>=250){
                robot.setAction(SpecimenWithIntakeDown);
                time4.reset();
            }
            if(gamepad1.dpad_left&&time5.milliseconds()>=250){
                robot.setAction(OuttakeGoToTransferSample);
                time5.reset();
            }
            if(gamepad1.dpad_right&&time1.milliseconds()>=250){
                robot.setAction(IntakeGoToTransfer);
                time1.reset();
            }
            if(gamepad1.right_bumper&&time2.milliseconds()>=250){
                robot.setAction(IntakeGoToTransferSpecimen);
                time2.reset();
            }
            if(gamepad2.dpad_up&&time1.milliseconds()>=250){
                robot.setAction(StartCollecting);
                time1.reset();
            }
            if(gamepad2.dpad_down&&time2.milliseconds()>=250){
                robot.setAction(StartCollectingWithWait);
                time2.reset();
            }
            if(gamepad2.dpad_left&&time3.milliseconds()>=250){
                robot.setAction(StartCollectingWithExtension);
                time3.reset();
            }
            if(gamepad2.dpad_right&&time4.milliseconds()>=250){
                robot.setAction(StartCollectingSpecific);
                time4.reset();
            }
            robot.drive.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            robot.drive.update();
            telemetry.addData("target lift: ", robot.lift.getTarget());
            telemetry.addData("current position lift: ", robot.lift.getCurrentPosition());

            telemetry.update();
            robot.update();
        }

    }
}
