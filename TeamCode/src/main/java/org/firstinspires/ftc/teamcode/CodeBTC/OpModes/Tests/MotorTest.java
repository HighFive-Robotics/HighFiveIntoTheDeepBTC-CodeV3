package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorTest extends LinearOpMode {

    DcMotorEx Motor1,Motor2,Motor3,Motor4;

    @Override
    public void runOpMode() throws InterruptedException {
        Motor1 = hardwareMap.get(DcMotorEx.class,"RBM"); //
        Motor2 = hardwareMap.get(DcMotorEx.class,"LFM"); //
        Motor3 = hardwareMap.get(DcMotorEx.class,"LBM"); //
        Motor4 = hardwareMap.get(DcMotorEx.class,"RFM"); //

        waitForStart();

        while(opModeIsActive()){
            Motor1.setPower(0);
            Motor2.setPower(0);
            Motor3.setPower(0);
            Motor4.setPower(0);
            if(gamepad1.a){
                Motor1.setPower(1);
            }
            if(gamepad1.b){
                Motor2.setPower(1);
            }
            if(gamepad1.x){
                Motor3.setPower(1);
            }
            if(gamepad1.y){
                Motor4.setPower(1);
            }

            telemetry.addData("Motor 1(a on xbox): ", Motor1.getPower());
            telemetry.addData("Motor 2(b on xbox): ", Motor2.getPower());
            telemetry.addData("Motor 3(x on xbox): ", Motor3.getPower());
            telemetry.addData("Motor 4(y on xbox): ", Motor4.getPower());
        }
    }
}
