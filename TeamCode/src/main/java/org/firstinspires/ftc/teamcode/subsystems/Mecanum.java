package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Mecanum {
    GoBildaPinpointDriver pinpoint;

    static boolean alliance; // if 1 blue else red
    static boolean up; // 1 if front side up else back side up
    public static int x;
    public static int y;

    public int shoot_point_x1;
    public int shoot_point_x2;
    public int shoot_point_y1;
    public int shoot_point_y2;




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
}
