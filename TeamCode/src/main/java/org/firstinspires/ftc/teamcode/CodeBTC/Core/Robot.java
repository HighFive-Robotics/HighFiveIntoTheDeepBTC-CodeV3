package org.firstinspires.ftc.teamcode.CodeBTC.Core;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.armSpitPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.transferPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.slidesRetractedPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.wristSpitPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.wristTransferPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.armCollectSpecimenPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.armInitPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Claw.clawClosePose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Claw.clawOpenPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.LinkageOuttake.slidesOuttakeRetractedPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.waitToComeForTransfer;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.wristCollectSpecimenPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CodeBTC.Constants;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.ArmIntake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.LinearSlides;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.WristIntake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Others.Climb;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Others.Led;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake.ArmOuttake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake.Claw;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake.Lift;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake.LinkageOuttake;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake.WristOuttake;

import java.util.List;

public class Robot implements HighModuleSimple {

    Telemetry telemetry;
    public PinpointLocalizer localizer;
    Led led;
    public ArmOuttake armOuttake;
    public LinkageOuttake linkageOuttake;
    public WristOuttake wristOuttake;
    public Claw claw;
    public Lift lift;
    public ActiveIntake activeIntake;
    public ArmIntake armIntake;
    public WristIntake wristIntake;
    public LinearSlides slides;
    public Climb climb;
    public Actions lastAction = Actions.None;
    public Follower drive;
    public Drive teleOpDrive;
    public List<LynxModule> allHubs;
    protected HardwareMap hardwareMap;

    boolean isAuto;

    ElapsedTime failSafeTimer = new ElapsedTime();

    boolean goToCollectSpecimen = false, finishCollectSpecimen = false, goToScoreSpecimen = false, finishScoreSpecimen = false;
    boolean setOuttakeForTransfer = false, liftGoToCollectSample = false, intakeGoToTransferSample = false, intakeGoToTransferSpecimen = false;
    boolean setIntakeForCollecting1 = false, setIntakeForCollecting2 = false, extendSlidesForCollecting2 = false, setIntakeForCollecting3 = false, setIntakeForCollecting4 = false;
    boolean shouldMakeTransferSample = false, getOuttakeToScoreSampleTransfer = false, failSafeTransfer = false, finishOuttakeScoreSampleTransfer = false, extendOuttakeTransferSample = false;

    public enum Actions {
        GoToCollectSpecimen,
        GoToScoreSpecimen,
        SpecimenWithIntakeUp,
        SpecimenWithIntakeDown,
        IntakeGoToTransfer,
        IntakeGoToTransferSpecimen,
        StartCollecting,
        StartCollectingWithWait,
        StartCollectingWithExtension,
        StartCollectingSpecific,
        OuttakeGoToTransferSample,
        TransferSample,
        CollectSampleFromPerimeter,
        None
    }

    public Robot(HardwareMap hardwareMap, Pose startPose, boolean isAuto, Constants.Color allianceColor, Telemetry telemetry){
        localizer = new PinpointLocalizer(hardwareMap , startPose);
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.isAuto = isAuto;
        if (isAuto) {
            claw = new Claw(hardwareMap, clawClosePose, true);
            armOuttake = new ArmOuttake(hardwareMap, armInitPose, true);
            wristOuttake = new WristOuttake(hardwareMap, waitToComeForTransfer, true);
            armIntake = new ArmIntake(hardwareMap, transferPose, true);
            wristIntake = new WristIntake(hardwareMap, wristTransferPose,true);
            drive = new Follower(hardwareMap,localizer, Constants.FConstants.class, Constants.LConstants.class);
            drive.setStartingPose(startPose);
        } else {
            claw = new Claw(hardwareMap, clawOpenPose, false);
            armOuttake = new ArmOuttake(hardwareMap, armCollectSpecimenPose, false);
            wristOuttake = new WristOuttake(hardwareMap, wristCollectSpecimenPose, false);
            armIntake = new ArmIntake(hardwareMap, armSpitPose, false);
            wristIntake = new WristIntake(hardwareMap, wristSpitPose,false);
            teleOpDrive = new Drive(hardwareMap);
        }
        linkageOuttake = new LinkageOuttake(hardwareMap, slidesOuttakeRetractedPose, isAuto);
        activeIntake = new ActiveIntake(hardwareMap,allianceColor);
        lift = new Lift(hardwareMap);
        climb = new Climb(hardwareMap);
        slides = new LinearSlides(hardwareMap, slidesRetractedPose, isAuto);
        led = new Led(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void setAction(Actions action) {
        switch (action) {
            case GoToCollectSpecimen:
                claw.setState(Claw.States.Open);
                linkageOuttake.setState(LinkageOuttake.States.Retracted, 300);
                goToCollectSpecimen = true;
                break;
            case GoToScoreSpecimen:
                claw.setState(Claw.States.Closed, 150);
                goToScoreSpecimen = true;
                break;
            case SpecimenWithIntakeUp:
                if(lift.getState() != Lift.States.Collect){
                    setAction(Actions.GoToCollectSpecimen);
                } else {
                    setAction(Actions.GoToScoreSpecimen);
                    armIntake.setState(ArmIntake.States.SpecimenWait);
                    wristIntake.setState(WristIntake.States.SpecimenWait);
                }
                break;
            case SpecimenWithIntakeDown:
                if(lift.getState() != Lift.States.Collect){
                    setAction(Actions.GoToCollectSpecimen);
                } else {
                    setAction(Actions.GoToScoreSpecimen);
                }
                break;
            case OuttakeGoToTransferSample:
                linkageOuttake.setState(LinkageOuttake.States.Retracted, 150);
                setOuttakeForTransfer = true;
            break;
            case IntakeGoToTransfer:
                armIntake.setState(ArmIntake.States.Transfer, 250);
                intakeGoToTransferSample = true;
                break;
            case IntakeGoToTransferSpecimen:
                armIntake.setState(ArmIntake.States.Spit, 250);
                intakeGoToTransferSpecimen = true;
                break;
            case StartCollecting:
                slides.setState(LinearSlides.States.Extended, 200);
                setIntakeForCollecting1 = true;
                break;
            case StartCollectingWithExtension:
                if (slides.getState() != LinearSlides.States.Aux && slides.getState() != LinearSlides.States.Custom) {
                    slides.setState(LinearSlides.States.Aux, 200);
                    activeIntake.setState(ActiveIntake.States.Wait);
                    setIntakeForCollecting2 = true;
                } else if (slides.getState() == LinearSlides.States.Aux || slides.getState() == LinearSlides.States.Custom) {
                    armIntake.setState(ArmIntake.States.Collect, 300);
                    wristIntake.setState(WristIntake.States.Collect);
                    activeIntake.setState(ActiveIntake.States.Collect);
                    extendSlidesForCollecting2 = true;
                }
                break;
            case StartCollectingWithWait: {
                if (slides.getState() != LinearSlides.States.Extended && slides.getState() != LinearSlides.States.Custom) {
                    slides.setState(LinearSlides.States.Extended, 200);
                    setIntakeForCollecting3 = true;
                } else if (armIntake.getState() == ArmIntake.States.WaitCollecting) {
                    armIntake.setState(ArmIntake.States.Collect);
                    wristIntake.setState(WristIntake.States.Collect);
                    activeIntake.setState(ActiveIntake.States.Collect);
                } else {
                    armIntake.setState(ArmIntake.States.WaitCollecting);
                    wristIntake.setState(WristIntake.States.Collect);
                    activeIntake.setState(ActiveIntake.States.Wait);
                }
            }
            break;
            case StartCollectingSpecific: {
                if (slides.getState() != LinearSlides.States.Extended && slides.getState() != LinearSlides.States.Custom) {
                    slides.setState(LinearSlides.States.Extended, 200);
                    setIntakeForCollecting4 = true;
                } else if (armIntake.getState() == ArmIntake.States.WaitSpecificOne && wristIntake.getState() == WristIntake.States.WaitCollectSpecificOne) {
                    armIntake.setState(ArmIntake.States.CollectSpecificOne);
                    wristIntake.setState(WristIntake.States.CollectSpecificOne);
                    activeIntake.setState(ActiveIntake.States.Collect);
                } else {
                    armIntake.setState(ArmIntake.States.WaitSpecificOne);
                    wristIntake.setState(WristIntake.States.WaitCollectSpecificOne);
                    activeIntake.setState(ActiveIntake.States.Wait);
                }
            }
            break;
            case TransferSample:
                claw.setState(Claw.States.ClosedTransfer, 150);
                activeIntake.setState(ActiveIntake.States.Collect, false);
                shouldMakeTransferSample = true;
                break;
        }
    }

    @Override
    public void update() {
        if(isAuto){
            if(drive.isLocalizationNAN()){
                activeIntake.intake.setPower(0);
                lift.motorsSetPower(0);
                drive.setMaxPower(0);
                localizer.resetIMU();
                Pose resetPose = localizer.getPose();
                localizer = null;
                localizer = new PinpointLocalizer(hardwareMap , resetPose);
                drive = null;
                drive = new Follower(hardwareMap,localizer, Constants.FConstants.class, Constants.LConstants.class);
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }
                return;
            }
        } else {
            localizer.update();
        }

        Constants.Globals.voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        if(goToCollectSpecimen && linkageOuttake.atTarget() && linkageOuttake.getState() == LinkageOuttake.States.Retracted){
            armOuttake.setState(ArmOuttake.States.CollectSpecimen, 450);
            wristOuttake.setState(WristOuttake.States.CollectSpecimen);
            goToCollectSpecimen = false;
            finishCollectSpecimen = true;
        }
        if(finishCollectSpecimen && armOuttake.atTarget()){
            lift.setState(Lift.States.Collect);
            finishCollectSpecimen = false;
        }
        if(goToScoreSpecimen && claw.atTarget()){
            lift.setState(Lift.States.HighChamber);
            armOuttake.setState(ArmOuttake.States.ScoreSpecimen, 300);
            wristOuttake.setState(WristOuttake.States.ScoreSpecimen);
            goToScoreSpecimen = false;
            finishScoreSpecimen = true;
        }
        if(finishScoreSpecimen && armOuttake.atTarget()){
            linkageOuttake.setState(LinkageOuttake.States.Extended);
            finishScoreSpecimen = false;
        }
        if(setOuttakeForTransfer && linkageOuttake.atTarget()){
            armOuttake.setState(ArmOuttake.States.Transfer, 450);
            wristOuttake.setState(WristOuttake.States.Transfer);
            claw.setState(Claw.States.Open);
            setOuttakeForTransfer = false;
            liftGoToCollectSample = true;
        }
        if(liftGoToCollectSample && armOuttake.atTarget()){
            lift.setState(Lift.States.Collect);
            liftGoToCollectSample = false;
        }
        if(intakeGoToTransferSample && armIntake.atTarget()){
            wristIntake.setState(WristIntake.States.Transfer);
            slides.setState(LinearSlides.States.Retracted);
            activeIntake.setState(ActiveIntake.States.Wait);
            intakeGoToTransferSample = false;
        }
        if(intakeGoToTransferSpecimen && armIntake.atTarget()){
            wristIntake.setState(WristIntake.States.Spit);
            slides.setState(LinearSlides.States.Retracted);
            intakeGoToTransferSpecimen = false;
        }
        if(setIntakeForCollecting1 && slides.atTarget()){
            armIntake.setState(ArmIntake.States.Collect);
            wristIntake.setState(WristIntake.States.Collect);
            activeIntake.setState(ActiveIntake.States.Collect);
            setIntakeForCollecting1 = false;
        }
        if(setIntakeForCollecting2 && slides.atTarget()){
            armIntake.setState(ArmIntake.States.WaitCollecting);
            wristIntake.setState(WristIntake.States.Collect);
            activeIntake.setState(ActiveIntake.States.Collect);
            setIntakeForCollecting2 = false;
        }
        if (extendSlidesForCollecting2 && armIntake.atTarget()) {
            slides.setState(LinearSlides.States.Extended);
            extendSlidesForCollecting2 = false;
        }
        if(setIntakeForCollecting3 && slides.atTarget()){
            armIntake.setState(ArmIntake.States.WaitCollecting);
            wristIntake.setState(WristIntake.States.Collect);
            setIntakeForCollecting3 = false;
        }
        if(setIntakeForCollecting4 && slides.atTarget()){
            armIntake.setState(ArmIntake.States.WaitSpecificOne);
            wristIntake.setState(WristIntake.States.WaitCollectSpecificOne);
            setIntakeForCollecting4 = false;
        }

        if (shouldMakeTransferSample && claw.atTarget()) {
            activeIntake.setState(ActiveIntake.States.HelpTransfer, false);
            lift.setState(Lift.States.HighBasket);
            shouldMakeTransferSample = false;
            getOuttakeToScoreSampleTransfer = true;
            failSafeTransfer = true;
            failSafeTimer.reset();
        }
        if (getOuttakeToScoreSampleTransfer && lift.getCurrentPosition() >= 200) {
            armOuttake.setState(ArmOuttake.States.ScoreSample, 750);
            wristOuttake.setState(WristOuttake.States.Sample);
            getOuttakeToScoreSampleTransfer = false;
            finishOuttakeScoreSampleTransfer = true;
        }
        if (failSafeTransfer && failSafeTimer.milliseconds() >= 400) {
            activeIntake.setState(ActiveIntake.States.Collect, false);
            failSafeTransfer = false;
        }
        if (finishOuttakeScoreSampleTransfer && lift.getCurrentPosition() >= 500) {
            activeIntake.setState(ActiveIntake.States.Wait);
            finishOuttakeScoreSampleTransfer = false;
            failSafeTransfer = false;
            extendOuttakeTransferSample = true;
        }
        if (extendOuttakeTransferSample && lift.getCurrentPosition() >= 500 && armOuttake.atTarget()) {
            linkageOuttake.setState(LinkageOuttake.States.Extended);
            extendOuttakeTransferSample = false;
        }

        activeIntake.updateColorAndDistance();
        activeIntake.update();
        if(isAuto){
            drive.update();
        }
        armOuttake.update();
        lift.update();
        claw.update();
        linkageOuttake.update();
        slides.update();
        armIntake.update();
        wristIntake.update();
        wristOuttake.update();
        climb.update();
        led.update();
        if(!isAuto){
            teleOpDrive.update();
        }
        for(LynxModule hub:allHubs){
            hub.clearBulkCache();
        }
    }

    public boolean isDone(){
        return !drive.isBusy();
    }
}
