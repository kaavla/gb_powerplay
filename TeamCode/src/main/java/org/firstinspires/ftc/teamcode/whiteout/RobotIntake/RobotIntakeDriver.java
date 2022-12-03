package org.firstinspires.ftc.teamcode.whiteout.RobotIntake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.whiteout.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.whiteout.OpenCVDetector.OpenCVDetectorDriver;
import org.firstinspires.ftc.teamcode.whiteout.OpenCVDetector.OpenCVDetectorHW;


public class RobotIntakeDriver implements Runnable{
    private RobotIntakeHW hw = new RobotIntakeHW();
    private OpenCVDetectorHW cameraHw = new OpenCVDetectorHW();
    boolean autoTurnOff = false;


    private double delta_x   = 0.0;
    private boolean is_done  = true;
    private boolean intake_on  = false;
    private double motor_power = 1.0;
    private double intake_power = motor_power;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotIntakeDriver(HardwareMap ahwMap, int threadSleepDelay, Telemetry t){
        hw.init(ahwMap);

        //Side Color does not matter for Intake camera. We just give Red as default
        cameraHw.init(ahwMap, OpenCVDetectorDriver.RobotCamera.INTAKE, tataAutonomousBase.SideColor.Red, t);

        sleepTime = threadSleepDelay;
        autoTurnOff = true;
    }

    private void robotIntakeAction(){

        if (is_done == false) {
            is_done = true;
            if (intake_on) {
                intakeSet(true, true);
            } else {
                intakeSet(false, false);;
            }
        }

        if (autoTurnOff && intake_on) {
            if (isElementCollected()){
                RobotLog.ii("SHANK", "AutoturnOFF - Basket full stopping motor");

                //Game element has been collected
                //Automatically reverse the intake
                hw.setPower(1*0.5);
                //hw.startIntake(motor_power);
                //intakeSet(true, false);

            }

        }
    }
    public boolean isElementCollected() {
        if (cameraHw.getLocation() == 1) {
            return true;
        }
        return false;
    }

    public RobotIntakeParams getRobotIntakeParams(){
        RobotIntakeParams param = new RobotIntakeParams();
        param.powerI0 = hw.getPower();
        return param;
    }

    public void toggleIntake(boolean reverse){
        if (is_done == true) {
            is_done = false;
            intake_on = !intake_on;
            if (reverse == true){
                intake_power = -1*motor_power;
            }else {
                intake_power = motor_power;
            }
        }
    }
    public void intakeSet(boolean isOn, boolean isCollecting){
        //manav function
        intake_on = isOn;
       if(!isOn){
           hw.stopIntake();
       }else{
           if(isCollecting){
               hw.startIntake(-1*motor_power);
           }else{
               hw.startIntake(motor_power);
           }
       }
    }
    public void checkGamePad(Gamepad gp) {
        autoTurnOff = true; //Manual Mode . dont use intake camera?

        if (gp.left_trigger > 0.5) {
            hw.setPower(intake_power);
            intake_on = true;
        } else if (gp.right_trigger > 0.5) {
            hw.setPower(-1*intake_power);
            intake_on = true;
        } else {
            hw.setPower(0.0);
            intake_on = false;
        }

    }

    public void stop(){ isRunning = false; }

    @Override
    public void run() {
        while(isRunning) {
            robotIntakeAction();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
