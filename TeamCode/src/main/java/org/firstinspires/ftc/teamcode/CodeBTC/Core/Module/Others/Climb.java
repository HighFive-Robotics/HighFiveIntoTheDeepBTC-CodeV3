package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Others;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.climbMotorLeftName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.climbMotorRightName;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighMotor;


public class Climb implements HighModuleSimple {
    public HighMotor leftClimbMotor, rightClimbMotor;

    public enum States {
        Climbing,
        Waiting,
        ResetLeft,
        ResetRight,
        ReverseClimb
    }
    States state = States.Waiting;
    public Climb(HardwareMap hardwareMap){
        leftClimbMotor = new HighMotor(hardwareMap.get(DcMotorEx.class , climbMotorLeftName) , HighMotor.RunMode.Time , false, true);
        rightClimbMotor = new HighMotor(hardwareMap.get(DcMotorEx.class , climbMotorRightName) , HighMotor.RunMode.Time , false, true);
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Waiting:
                leftClimbMotor.setPower(0);
                rightClimbMotor.setPower(0);
                break;
            case Climbing:
                leftClimbMotor.setPower(1);
                rightClimbMotor.setPower(1);
                break;
            case ResetLeft:
                leftClimbMotor.setPower(-1);
                break;
            case ResetRight:
                rightClimbMotor.setPower(-1);
                break;
            case ReverseClimb:
                leftClimbMotor.setPower(-1);
                rightClimbMotor.setPower(-1);
                break;
        }
    }
    public void setState(States state , double time){
        this.state = state;
        switch (state) {
            case Waiting:
                leftClimbMotor.setPower(0);
                rightClimbMotor.setPower(0);
                break;
            case Climbing:
                leftClimbMotor.setPower(1,time);
                rightClimbMotor.setPower(1,time);
                break;
            case ResetLeft:
                leftClimbMotor.setPower(-1,time);
                break;
            case ResetRight:
                rightClimbMotor.setPower(-1,time);
                break;
            case ReverseClimb:
                leftClimbMotor.setPower(-1,time);
                rightClimbMotor.setPower(-1,time);
                break;
        }
    }

    @Override
    public void update() {
        leftClimbMotor.update();
        rightClimbMotor.update();
        if(leftClimbMotor.getPower() == 0 && rightClimbMotor.getPower() == 0){
            state = States.Waiting;
        }
    }
}