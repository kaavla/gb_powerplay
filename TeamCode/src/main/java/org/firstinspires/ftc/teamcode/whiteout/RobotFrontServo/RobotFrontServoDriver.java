package org.firstinspires.ftc.teamcode.whiteout.RobotFrontServo;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class RobotFrontServoDriver implements Runnable{
    private RobotFrontServoHW hw = new RobotFrontServoHW();

    private double delta_x   = 0.0;
    private boolean is_done  = true;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotFrontServoDriver(HardwareMap ahwMap, int threadSleepDelay){
        hw.init(ahwMap);
        sleepTime = threadSleepDelay;
    }


    public RobotFrontServoParams getRobotIntakeParams(){
        RobotFrontServoParams param = new RobotFrontServoParams();
        param.pos = hw.getPos();
        return param;
    }

    public void triggerFrontservo(){
        hw.setPos(0.8);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        hw.resetFrontServo();
    }

    public void checkGamePad(Gamepad gp) {
        if (gp.right_stick_button) {
            triggerFrontservo();
        }
    }

    public void stop()
    { isRunning = false;
    }

    @Override
    public void run() {
        while(isRunning) {
            getRobotIntakeParams();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
