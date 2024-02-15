package org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;

public interface Red_Points_Overlap {

    /**
     * collect white pixels from stack
     * */

    /**first position*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(270));
    Vector2D CC1F = new Vector2D(getRealCoords(281), getRealCoords(196));
    Vector2D CE1F = new Vector2D(getRealCoords(96), getRealCoords(207));

    //second segment
    Vector2D CS2F = CE1F;
    Vector2D CC2F = new Vector2D(getRealCoords(20), getRealCoords(209));
    Vector2D CE2F = new Vector2D(getRealCoords(39), getRealCoords(264));

    /**
     * deliver points
     * */

    /**first position*/
    Vector2D DS1F = new Vector2D(CE2F.getX(), CE2F.getY());
    Vector2D DC1F = new Vector2D(getRealCoords(20), getRealCoords(209));
    Vector2D DE1F = new Vector2D(getRealCoords(96), getRealCoords(207));

    Vector2D DS2F = DE1F;
    Vector2D DC2F = new Vector2D(getRealCoords(290), getRealCoords(196));
    Vector2D DE2F = new Vector2D(getRealCoords(300), getRealCoords(270));

}
