package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinPoint  {

    GoBildaPinpointDriver pinpoint;

    static boolean alliance = true; // if 1 blue else red
    static boolean up; // 1 if front side up else back side up
    public static int x;
    public static int y;

    public int shoot_point_x1;
    public int shoot_point_x2;
    public int shoot_point_y1;
    public int shoot_point_y2;

    if(alliance){

        shoot_point_x1=48;
        shoot_point_x2=72;
        if(up){

            shoot_point_y1=0;
            shoot_point_y2=24;
        }
        else {

            shoot_point_y1=136;
            shoot_point_y2=144;

        }

    }
    else{

        shoot_point_x1=72;
        shoot_point_x2=96;
        if(up){

            shoot_point_y1=0;
            shoot_point_y2=36;
        }
        else {

            shoot_point_y1=136;
            shoot_point_y2=144;

        }

    }


    public void loop(){

        if(x>=shoot_point_x1 && x<=shoot_point_x2 && y>=shoot_point_y1 && y<=shoot_point_y2){
            //shoot
        }
        else {
            if(x<shoot_point_x1){
                // move to left
            }
            if(x>shoot_point_x2){
                // move to right
            }
            if(y<shoot_point_y1){
                //move upside
            }
            if(y>shoot_point_y2){
                // move downside
            }
        }
    }


    public void setPosition(Pose2D pose){
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }


}



