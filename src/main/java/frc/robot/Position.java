package frc.robot;

public class Position{
    public double x;
    public double y;

    public Position(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Position(Position clone){
        this.x = clone.x;
        this.y = clone.y;
    }
}