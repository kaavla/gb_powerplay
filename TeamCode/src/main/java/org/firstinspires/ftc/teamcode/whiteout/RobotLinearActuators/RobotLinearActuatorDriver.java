package org.firstinspires.ftc.teamcode.whiteout.RobotLinearActuators;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class RobotLinearActuatorDriver implements Runnable{


    private RobotLinearActuatorHW hw = new RobotLinearActuatorHW();

    private double delta_x   = 0.0;
    private boolean is_done  = true;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotLinearActuatorDriver(HardwareMap ahwMap, int threadSleepDelay, int cnt){
        if (cnt == 0) {
            hw.init(ahwMap, RobotLinearActuatorHW.Side.LEFT);
        }else{
            hw.init(ahwMap, RobotLinearActuatorHW.Side.RIGHT);
        }
        sleepTime = threadSleepDelay;
    }


    public RobotLinearActuatorParams getRobotLinearActuatorParams(){
        RobotLinearActuatorParams param = new RobotLinearActuatorParams();
        param.pos = hw.getPos();
        return param;
    }

    public void pullLinearActuatorBy(double dx) {
        if (is_done == true) {
            delta_x = dx;
            is_done = false;
        }
    }

    private void update() {
       if (delta_x != 0 ) {
            hw.pullUp(delta_x);
            delta_x = 0;
            is_done = true;
        }
    }

    public void stop()
    { isRunning = false;
    }

    @Override
    public void run() {
        while(isRunning) {
            update();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
