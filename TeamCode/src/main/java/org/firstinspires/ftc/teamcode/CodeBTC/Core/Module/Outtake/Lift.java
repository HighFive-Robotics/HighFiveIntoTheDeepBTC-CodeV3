package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.liftMotorLeftName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.liftMotorRightName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftCorrectionCoefficients.gravityGain;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftCorrectionCoefficients.liftPIDCoefficients_goingDown;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftCorrectionCoefficients.liftPIDCoefficients_goingUp;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftCorrectionCoefficients.maxCurrentDrawn;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftPositions.liftCollect;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftPositions.liftHighBasket;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftPositions.liftHighBasketSpecial;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftPositions.liftHighChamber;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftPositions.liftLowBasket;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.LiftPositions.liftLowChamber;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighMotor;

public class Lift implements HighModuleSimple {

    public HighMotor leftMotor, rightMotor;
    boolean shouldResetEnc = false;
    private double power = 0;
    private boolean newTarget = false;
    ElapsedTime timerToResetEnc = new ElapsedTime();
    public States state = States.Collect;

    public enum States {
        Collect,
        HighBasket,
        HighBasketSpecial,
        LowBasket,
        HighChamber,
        LowChamber,
        Reset
    }

    public Lift(HardwareMap hardwareMap){
        leftMotor = new HighMotor(hardwareMap.get(DcMotorEx.class, liftMotorLeftName), HighMotor.RunMode.Standard, false, true);
        rightMotor = new HighMotor(hardwareMap.get(DcMotorEx.class, liftMotorRightName), HighMotor.RunMode.PID, false, true, true, false);
        rightMotor.setPIDCoefficients(liftPIDCoefficients_goingUp.p, liftPIDCoefficients_goingUp.i, liftPIDCoefficients_goingUp.d, gravityGain, HighMotor.FeedForwardType.Lift);
        rightMotor.setTarget(0);
        rightMotor.setTolerance(20);
    }

    public void setState(States state) {
        this.state = state;
        newTarget = true;
        switch (state) {
            case Collect:
                power = -0.9;
                motorsSetPower(power);
                rightMotor.setRunMode(HighMotor.RunMode.Standard);
                rightMotor.setTarget(liftCollect);
                break;
            case LowBasket:
                rightMotor.setTarget(liftLowBasket);
                break;
            case HighBasket:
                rightMotor.setTarget(liftHighBasket);
                break;
            case HighChamber:
                rightMotor.setTarget(liftHighChamber);
                break;
            case LowChamber:
                rightMotor.setTarget(liftLowChamber);
                break;
            case HighBasketSpecial:
                rightMotor.setTarget(liftHighBasketSpecial);
                break;
            case Reset:
                power = -1;
                rightMotor.setRunMode(HighMotor.RunMode.Standard);
                motorsSetPower(power);
                break;
        }
    }

    public void resetEnc() {
        motorsSetPower(0);
        rightMotor.setTarget(0);
        leftMotor.resetMotor();
        rightMotor.resetMotor();
        rightMotor.resetPID();
        state = States.Collect;
    }

    public void setTarget(double target) {
        rightMotor.setTarget(target);
    }

    public boolean atTarget() {
        return rightMotor.atTarget();
    }

    public double getTarget() {
        return rightMotor.getTarget();
    }

    public double getPower(double currentPosition) {
        return rightMotor.getPower();
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        if (state != States.Reset) {//todo make voltage work
            if (rightMotor.getCurrentDrawn() * (12.0 / 12.0) >= (maxCurrentDrawn) && rightMotor.getCurrentPosition() <= 20 && rightMotor.getRunMode() == HighMotor.RunMode.Standard) {
                power = 0;
                motorsSetPower(power);
                shouldResetEnc = true;
                timerToResetEnc.reset();
            }
            if (timerToResetEnc.milliseconds() >= 100 && shouldResetEnc && rightMotor.getCurrentDrawn() <= 2) {
                shouldResetEnc = false;
                rightMotor.setPIDCoefficients(liftPIDCoefficients_goingUp.p, liftPIDCoefficients_goingUp.i, liftPIDCoefficients_goingUp.d, gravityGain, HighMotor.FeedForwardType.Lift,1);
                resetEnc();
            }
            if (newTarget) {
                if (rightMotor.getCurrentPosition() > getTarget())
                    rightMotor.setPIDCoefficients(liftPIDCoefficients_goingDown.p, liftPIDCoefficients_goingDown.i, liftPIDCoefficients_goingDown.d, gravityGain, HighMotor.FeedForwardType.Lift,1);
                else
                    rightMotor.setPIDCoefficients(liftPIDCoefficients_goingUp.p, liftPIDCoefficients_goingUp.i, liftPIDCoefficients_goingUp.d, gravityGain, HighMotor.FeedForwardType.Lift,1);
            }
            if (rightMotor.getRunMode() == HighMotor.RunMode.PID) {
                power = rightMotor.getPower();
                motorsSetPower(power);
            }
        }
    }

    public void motorsSetPower(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
