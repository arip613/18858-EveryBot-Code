/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Prism;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Utility class for smoothly tweening between two colors over time
 */
public class ColorTween {
    private Color startColor;
    private Color endColor;
    private double durationSeconds;
    private ElapsedTime timer;
    private boolean isActive;

    public ColorTween() {
        timer = new ElapsedTime();
        isActive = false;
    }

    /**
     * Start a color tween animation
     * @param start Starting color
     * @param end Ending color
     * @param duration Duration in seconds
     */
    public void start(Color start, Color end, double duration) {
        this.startColor = start;
        this.endColor = end;
        this.durationSeconds = duration;
        this.timer.reset();
        this.isActive = true;
    }

    /**
     * Get the current interpolated color
     * @return Current color based on elapsed time
     */
    public Color getCurrentColor() {
        if (!isActive) {
            return startColor;
        }

        double elapsed = timer.seconds();
        
        // Check if animation is complete
        if (elapsed >= durationSeconds) {
            isActive = false;
            return endColor;
        }

        // Calculate progress (0.0 to 1.0)
        double progress = elapsed / durationSeconds;

        // Linear interpolation between start and end colors
        return lerp(startColor, endColor, progress);
    }

    /**
     * Linear interpolation between two colors
     * @param start Starting color
     * @param end Ending color
     * @param t Progress value (0.0 to 1.0)
     * @return Interpolated color
     */
    private Color lerp(Color start, Color end, double t) {
        int red = (int)(start.red + (end.red - start.red) * t);
        int green = (int)(start.green + (end.green - start.green) * t);
        int blue = (int)(start.blue + (end.blue - start.blue) * t);
        
        return new Color(red, green, blue);
    }

    /**
     * Check if the tween is currently active
     * @return true if animating, false if complete
     */
    public boolean isActive() {
        return isActive;
    }

    /**
     * Stop the current tween
     */
    public void stop() {
        isActive = false;
    }

    /**
     * Get progress of current tween (0.0 to 1.0)
     * @return Current progress
     */
    public double getProgress() {
        if (!isActive) return 1.0;
        return Math.min(timer.seconds() / durationSeconds, 1.0);
    }
}
