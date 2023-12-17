package org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.jetbrains.annotations.NotNull;

import java.util.Locale;

@SuppressWarnings("unused")
public class robotPos {
	private double x, y, heading;

	public robotPos(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
	}

	/**
	 * constructs a new default vector with values of 0 for both x and y
	 */
	public robotPos() {
		this(0, 0, 0);
	}


	public double getX() {
		return x;
	}

	/**
	 * mutates state
	 *
	 * @param x
	 */
	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public double getHeading() {
		return heading;
	}

	/**
	 * mutates state
	 *
	 * @param y
	 */
	public void setY(double y) {
		this.y = y;
	}

	public void setHeading(double heading) {
		this.heading = heading;
	}


	public double getMagnitude() {
		return Math.hypot(x, y);
	}

	/**
	 * mutates state
	 *
	 * @param x
	 * @param y
	 * @return self
	 */
	public robotPos set(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
		return this;
	}


	@Override
	public boolean equals(@Nullable @org.jetbrains.annotations.Nullable Object obj) {
		if (!(obj instanceof robotPos)) return false;
		robotPos other = (robotPos) obj;
		return this.getX() == other.getX() && this.getY() == other.getY();
	}

	@NonNull
	@NotNull
	@Override
	public String toString() {
		return String.format(Locale.ENGLISH, "x: %f, y: %f", getX(), getY());
	}
}
