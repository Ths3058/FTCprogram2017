package org.firstinspires.ftc.teamcode;

/**
 * Created by Robotics.
 *
 * @author Ryan Kirkpatrick
 * @version 11/10/2016
 */

class StaticFunctions {
    //--------------------------------------------------------------------------
    // distToEnc ()
    // Parameters inches
    // Return encoder count
    //--------------------------------------------------------------------------
    static int distToEnc(double inch) { return (int)(inch/12.0*900); } //2750

    //--------------------------------------------------------------------------
    // degreesToEnc ()
    // Parameters degrees
    // Return encoder count
    //--------------------------------------------------------------------------
    static int degreesToEnc(int degrees) { return (int)(degrees/90.0*1300); } // 2860, 2500
}
