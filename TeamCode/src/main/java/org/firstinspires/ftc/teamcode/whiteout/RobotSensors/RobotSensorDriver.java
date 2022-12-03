package org.firstinspires.ftc.teamcode.whiteout.RobotSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotSensorDriver implements Runnable {
    private RobotSensorHW sensorHW = new RobotSensorHW();

    private RobotSensorParams rsp = new RobotSensorParams();

    private static final int MAX_FILTER_SIZE = 1;
    private RobotSensorParams[] rspAray = new RobotSensorParams[MAX_FILTER_SIZE];
    private int arrIdx = 0;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotSensorDriver(HardwareMap ahwMap, int threadSleepDelay, Telemetry t) {
        sensorHW.init(ahwMap, t);
        sleepTime = threadSleepDelay;
    }

    public int getBarCodeRED() {
        //Deprecated
        RobotSensorParams avg_rsp = getAvg();

        int barCodeLoc = 1;
        //check the left front sensors
        double detect_dist = 6.0;

        if ((avg_rsp.x_LF < detect_dist) ) {
            barCodeLoc = 1;
        } else if ((avg_rsp.x_RF < detect_dist)) {
            barCodeLoc = 2;
        } else {
            barCodeLoc = 3;
        }
        return barCodeLoc;
    }
    public int getBarCodeBLUE() {
        //Deprecated
        RobotSensorParams avg_rsp = getAvg();
        int barCodeLoc = 1;
        double detect_dist = 6.0;
        //check the left front sensors
        if ((avg_rsp.x_RF < detect_dist) ) {
            barCodeLoc = 3;
        } else if ((avg_rsp.x_LF < detect_dist)) {
            barCodeLoc = 2;
        } else {
            barCodeLoc = 1;
        }
        return barCodeLoc;
    }

    private void robotSensorUpdate() {
        rsp = sensorHW.getDistances();
        addToFilter(rsp);
    }

    public RobotSensorParams getRobotSensorParams(){
        rsp = getAvg();
        return rsp;
    }

    public void addToFilter(RobotSensorParams in) {
        rspAray[arrIdx] = in;
        arrIdx = arrIdx + 1;
        if (arrIdx >= MAX_FILTER_SIZE) {
            arrIdx = 0;
        }
    }

    public RobotSensorParams getAvg(){
        RobotSensorParams avg = new RobotSensorParams();
        /*
        avg.x_LF = 0;
        avg.x_RF = 0;
        avg.x_LR = 0;
        avg.x_RR = 0;
        avg.x_LS = 0;
        avg.x_RS = 0;
        for (int i = 0; i < MAX_FILTER_SIZE; i++) {
            avg.x_LF += rspAray[i].x_LF;
            avg.x_RF += rspAray[i].x_RF;
            avg.x_LR += rspAray[i].x_LR;
            avg.x_RR += rspAray[i].x_RR;
            avg.x_LS += rspAray[i].x_LS;
            avg.x_RS += rspAray[i].x_RS;
        }

         */
        avg.x_LF = rsp.x_LF/MAX_FILTER_SIZE;
        avg.x_RF = rsp.x_RF/MAX_FILTER_SIZE;
        avg.x_LR = rsp.x_LR/MAX_FILTER_SIZE;
        avg.x_RR = rsp.x_RR/MAX_FILTER_SIZE;
        avg.x_LS = rsp.x_LS/MAX_FILTER_SIZE;
        avg.x_RS = rsp.x_RS/MAX_FILTER_SIZE;
        avg.c_LS = rsp.c_LS;
        avg.c_RS = rsp.c_RS;

        return avg;
    }


    public void stop() {
        isRunning = false;
    }

    @Override
    public void run() {
        while (isRunning) {
            robotSensorUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
