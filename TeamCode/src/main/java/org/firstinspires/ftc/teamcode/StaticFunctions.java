package org.firstinspires.ftc.teamcode;

/**
 * Created by ryank on 11/10/2016.
 */

public class StaticFunctions {
    //--------------------------------------------------------------------------
    // distToEnc ()
    // Parameters inches
    // Return encoder count
    //--------------------------------------------------------------------------
    static int distToEnc(double inch) { return (int)(inch/12.0*2500); } //2750

    //--------------------------------------------------------------------------
    // degreesToEnc ()
    // Parameters degrees
    // Return encoder count
    //--------------------------------------------------------------------------
    static int degreesToEnc(int degrees) { return (int)(degrees/90.0*1800); } // 2860, 2500
}
