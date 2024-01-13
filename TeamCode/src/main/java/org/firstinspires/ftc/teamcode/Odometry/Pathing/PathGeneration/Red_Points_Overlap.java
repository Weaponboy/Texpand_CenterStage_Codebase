package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;

public interface Red_Points_Overlap {

    /**
     * collect white pixels from stack
     * */

    /**first position*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(275));
    Vector2D CC1F = new Vector2D(getRealCoords(294), getRealCoords(211));
    Vector2D CE1F = new Vector2D(getRealCoords(181), getRealCoords(167));

    //second segment
    Vector2D CS2F = CE1F;
    Vector2D CC2F = new Vector2D(getRealCoords(30), getRealCoords(111));
    Vector2D CE2F = new Vector2D(getRealCoords(41), getRealCoords(225));

    /**second position*/
    Vector2D CS1S = new Vector2D(getRealCoords(300), getRealCoords(275));
    Vector2D CC1S = new Vector2D(getRealCoords(294), getRealCoords(211));
    Vector2D CE1S = new Vector2D(getRealCoords(181), getRealCoords(167));

    //second segment
    Vector2D CS2S = CE1S;
    Vector2D CC2S = new Vector2D(getRealCoords(30), getRealCoords(111));
    Vector2D CE2S = new Vector2D(getRealCoords(41), getRealCoords(225));

    /**third position*/
    Vector2D CS1T = new Vector2D(getRealCoords(300), getRealCoords(275));
    Vector2D CC1T = new Vector2D(getRealCoords(294), getRealCoords(211));
    Vector2D CE1T = new Vector2D(getRealCoords(181), getRealCoords(167));

    //second segment
    Vector2D CS2T = CE1T;
    Vector2D CC2T = new Vector2D(getRealCoords(30), getRealCoords(111));
    Vector2D CE2T = new Vector2D(getRealCoords(41), getRealCoords(225));

    /**
     * deliver points
     * */

    /**first position*/
    Vector2D DS1F = new Vector2D(CE2F.getX(), CE2F.getY());
    Vector2D DC1F = new Vector2D(getRealCoords(84), getRealCoords(171));
    Vector2D DE1F = new Vector2D(getRealCoords(179), getRealCoords(184));

    Vector2D DS2F = DE1F;
    Vector2D DC2F = new Vector2D(getRealCoords(293), getRealCoords(200));
    Vector2D DE2F = new Vector2D(getRealCoords(296), getRealCoords(255));

    /**second position*/
    Vector2D DS1S = new Vector2D(CE2S.getX(), CE2S.getY());
    Vector2D DC1S = new Vector2D(getRealCoords(84), getRealCoords(171));
    Vector2D DE1S = new Vector2D(getRealCoords(179), getRealCoords(184));

    Vector2D DS2S = new Vector2D(DE1S.getX(), DE1S.getY());
    Vector2D DC2S = new Vector2D(getRealCoords(293), getRealCoords(200));
    Vector2D DE2S = new Vector2D(getRealCoords(296), getRealCoords(275));

    /**third position*/
    Vector2D DS1T = new Vector2D(CE2T.getX(), CE2T.getY());
    Vector2D DC1T = new Vector2D(getRealCoords(84), getRealCoords(171));
    Vector2D DE1T = new Vector2D(getRealCoords(179), getRealCoords(184));

    Vector2D DS2T = new Vector2D(DE1T.getX(), DE1T.getY());
    Vector2D DC2T = new Vector2D(getRealCoords(293), getRealCoords(200));
    Vector2D DE2T = new Vector2D(getRealCoords(296), getRealCoords(290));


}
