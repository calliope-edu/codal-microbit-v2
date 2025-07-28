/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "MicroBitConfig.h"
#include "MicroBitCompassCalibrator.h"
#include "EventModel.h"

using namespace codal;

#define CALIBRATION_INCREMENT     200

/**
  * Constructor.
  *
  * Create an object capable of calibrating the compass.
  *
  * The algorithm uses an accelerometer to ensure that a broad range of sample data has been gathered
  * from the compass module, then performs a least mean squares optimisation of the
  * results to determine the calibration data for the compass.
  *
  * The LED matrix display is used to provide feedback to the user on the gestures required.
  *
  * @param compass The compass instance to calibrate.
  *
  * @param accelerometer The accelerometer to gather contextual data from.
  *
  * @param display The LED matrix to display user feedback on.
  */
MicroBitCompassCalibrator::MicroBitCompassCalibrator(Compass& _compass, Accelerometer& _accelerometer, MicroBitDisplay& _display) : compass(_compass), accelerometer(_accelerometer), display(_display)
{
    this->storage = NULL;

    if (EventModel::defaultEventBus)
        EventModel::defaultEventBus->listen(MICROBIT_ID_COMPASS, MICROBIT_COMPASS_EVT_CALIBRATE, this, &MicroBitCompassCalibrator::calibrateUX, MESSAGE_BUS_LISTENER_IMMEDIATE);
}

/**
 * Constructor.
 *
 * Create an object capable of calibrating the compass.
 *
 * The algorithm uses an accelerometer to ensure that a broad range of sample data has been gathered
 * from the compass module, then performs a least mean squares optimisation of the
 * results to determine the calibration data for the compass.
 *
 * The LED matrix display is used to provide feedback to the user on the gestures required.
 *
 * @param compass The compass instance to calibrate.
 * @param accelerometer The accelerometer to gather contextual data from.
 * @param display The LED matrix to display user feedback on.
 * @param storage The object to use for storing calibration data in persistent FLASH. 
 */
MicroBitCompassCalibrator::MicroBitCompassCalibrator(Compass& _compass, Accelerometer& _accelerometer, MicroBitDisplay& _display, MicroBitStorage &storage) : compass(_compass), accelerometer(_accelerometer), display(_display)
{
    this->storage = &storage;

    //Attempt to load any stored calibration datafor the compass.
    KeyValuePair *calibrationData =  this->storage->get("compassCal");

    if(calibrationData != NULL)
    {
        CompassCalibration cal;
        memcpy(&cal, calibrationData->value, sizeof(CompassCalibration));
        compass.setCalibration(cal);
        delete calibrationData;
    }

    if (EventModel::defaultEventBus)
        EventModel::defaultEventBus->listen(MICROBIT_ID_COMPASS, MICROBIT_COMPASS_EVT_CALIBRATE, this, &MicroBitCompassCalibrator::calibrateUX, MESSAGE_BUS_LISTENER_IMMEDIATE);
}
/**
 * Scoring function for a hill climb algorithm.
 *
 * @param c An approximated centre point
 * @param data a collection of data points
 * @param samples the number of samples in the 'data' array
 *
 * @return The deviation between the closest and further point in the data array from the point given. 
 */
float MicroBitCompassCalibrator::measureScore(Sample3D &c, Sample3D *data, int samples)
{
    float minD;
    float maxD;

    minD = maxD = c.dSquared(data[0]);
    for (int i = 1; i < samples; i++)
    {
        float d = c.dSquared(data[i]);

        if (d < minD)
            minD = d;

        if (d > maxD)
            maxD = d;
    }

    return (maxD - minD);
}

/**
 * Calculates an independent X, Y, Z scale factor and centre for a given set of data points, assumed to be on
 * a bounding sphere
 *
 * @param data An array of all data points
 * @param samples The number of samples in the 'data' array.
 *
 * This algorithm should be called with no fewer than 12 points, but testing has indicated >21 points provides
 * a more robust calculation.
 *
 * @return A calibration structure containing the a calculated centre point, the radius of the minimum
 * enclosing spere of points and a scaling factor for each axis that places those points as close as possible
 * to the surface of the containing sphere.
 */
CompassCalibration MicroBitCompassCalibrator::calibrate(Sample3D *data, int samples)
{
    Sample3D centre = approximateCentre(data, samples);
    return spherify(centre, data, samples);
}
/**
 * Calculates an independent scale factor for X,Y and Z axes that places the given data points on a bounding sphere
 *
 * @param centre A proviously calculated centre point of all data.
 * @param data An array of all data points
 * @param samples The number of samples in the 'data' array.
 *
 * @return A calibration structure containing the centre point provided, the radius of the minimum
 * enclosing spere of points and a scaling factor for each axis that places those points as close as possible
 * to the surface of the containing sphere.
 */
CompassCalibration MicroBitCompassCalibrator::spherify(Sample3D centre, Sample3D *data, int samples)
{
    // First, determine the radius of the enclosing sphere from the given centre.
    // n.b. this will likely be different to the radius from the centre of mass previously calculated.
    // We use the same algorithm though.
    CompassCalibration result;

    float radius = 0;
    float scaleX = 0.0;
    float scaleY = 0.0;
    float scaleZ = 0.0;

    float scale = 0.0;
    float weightX = 0.0;
    float weightY = 0.0;
    float weightZ = 0.0;

    for (int i = 0; i < samples; i++)
    {
        int d = sqrtf((float)centre.dSquared(data[i]));

        if (d > radius)
            radius = d;
    }

    // Now, for each data point, determine a scalar multiplier for the vector between the centre and that point that
    // takes the point onto the surface of the enclosing sphere.
    for (int i = 0; i < samples; i++)
    {
        // Calculate the distance from this point to the centre of the sphere
        float d = sqrtf(centre.dSquared(data[i]));

        // Now determine a scalar multiplier that, when applied to the vector to the centre,
        // will place this point on the surface of the sphere.
        float s = (radius / d) - 1;

        if (scale < s)
            scale = s;

        // next, determine the scale effect this has on each of our components.
        float dx = (data[i].x - centre.x); 
        float dy = (data[i].y - centre.y);
        float dz = (data[i].z - centre.z);

        weightX += s * fabsf(dx / d);
        weightY += s * fabsf(dy / d);
        weightZ += s * fabsf(dz / d);
    }

    float wmag = sqrtf((weightX * weightX) + (weightY * weightY) + (weightZ * weightZ));

    scaleX = 1.0f + scale * (weightX / wmag);
    scaleY = 1.0f + scale * (weightY / wmag);
    scaleZ = 1.0f + scale * (weightZ / wmag);

    result.scale.x = (int)(1024 * scaleX);
    result.scale.y = (int)(1024 * scaleY);
    result.scale.z = (int)(1024 * scaleZ);

    result.centre.x = centre.x;
    result.centre.y = centre.y;
    result.centre.z = centre.z;

    result.radius = radius;

    return result;
}

/*
 * Performs an interative approximation (hill descent) algorithm to determine an
 * estimated centre point of a sphere upon which the given data points reside.
 *
 * @param data an array containing sample points
 * @param samples the number of sample points in the 'data' array.
 *
 * @return the approximated centre point of the points in the 'data' array.
 */
Sample3D MicroBitCompassCalibrator::approximateCentre(Sample3D *data, int samples)
{
    Sample3D c,t;
    Sample3D centre = { 0,0,0 };
    Sample3D best = { 0,0,0 };

    float score;

    for (int i = 0; i < samples; i++)
    {
        centre.x += data[i].x;
        centre.y += data[i].y;
        centre.z += data[i].z;
    }

    // Calclulate a centre of mass for our input samples. We only use this for validation purposes.
    centre.x = centre.x / samples;
    centre.y = centre.y / samples;
    centre.z = centre.z / samples;

    // Start hill climb in the centre of mass.
    c = centre;

    // calculate the nearest and furthest point to us.
    score = measureScore(c, data, samples);

    // iteratively attempt to improve position...
    while (1)
    {
        for (int x = -CALIBRATION_INCREMENT; x <= CALIBRATION_INCREMENT; x=x+CALIBRATION_INCREMENT)
        {
            for (int y = -CALIBRATION_INCREMENT; y <= CALIBRATION_INCREMENT; y=y+CALIBRATION_INCREMENT)
            {
                for (int z = -CALIBRATION_INCREMENT; z <= CALIBRATION_INCREMENT; z=z+CALIBRATION_INCREMENT)
                {
                    t = c;
                    t.x += x;
                    t.y += y;
                    t.z += z;

                    float s = measureScore(t, data, samples);
                    if (s < score)
                    {
                        score = s;
                        best = t;
                    }
                }
            }
        }

        if (best.x == c.x && best.y == c.y && best.z == c.z)
            break;

        c = best;
    }

    return c;
}

/**
 * Performs a simple game that in parallel, calibrates the compass.
 *
 * This function is executed automatically when the user requests a compass bearing, and compass calibration is required.
 *
 * This function is, by design, synchronous and only returns once calibration is complete.
 */
void MicroBitCompassCalibrator::calibrateUX(MicroBitEvent)
{
    struct Point
    {
        uint8_t x;
        uint8_t y;
    };

    const int PERIMETER_POINTS = 25;

    const int PIXEL1_THRESHOLD = 200;
    const int PIXEL2_THRESHOLD = 680;
    const int TIME_STEP = 80;

    target_wait(100);

    static const Point perimeter[PERIMETER_POINTS] = {{0,0}, {1,0}, {2,0}, {3,0}, {4,0}, {0,1}, {1,1}, {2,1}, {3,1}, {4,1}, {0,2}, {1,2}, {2,2}, {3,2}, {4,2}, {0,3}, {1,3}, {2,3}, {3,3}, {4,3}, {0,4}, {1,4}, {2,4}, {3,4}, {4,4}};
    Point cursor = {2,2};

    MicroBitImage img(5,5);
    MicroBitImage smiley("0,255,0,255,0\n0,255,0,255,0\n0,0,0,0,0\n255,0,0,0,255\n0,255,255,255,0\n");

    Sample3D data[PERIMETER_POINTS];
    uint8_t visited[PERIMETER_POINTS] = { 0 };
    uint8_t samples = 0;

    // Set the display to full brightness (in case a user program has it very dim / off). Store the current brightness, so we can restore it later.
    int displayBrightness = display.getBrightness();
    display.setBrightness(255);

    // Firstly, we need to take over the display. Ensure all active animations are paused.
    display.stopAnimation();

    while(samples < PERIMETER_POINTS)
    {
        // Define the animation path around the edge of the screen.
        static const Point animationPath[] = {
            {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0},
            {4, 1}, {4, 2}, {4, 3}, {4, 4},
            {3, 4}, {2, 4}, {1, 4}, {0, 4},
            {0, 3}, {0, 2}, {0, 1}
        };
        static const int animationPathLength = sizeof(animationPath) / sizeof(animationPath[0]);
        static int animationIndex = 0;

        img.clear();

        // Turn on any pixels that have been visited.
        for (int i = 0; i < PERIMETER_POINTS; i++)
        {
            if (visited[i])
                img.setPixelValue(perimeter[i].x, perimeter[i].y, 80); // Full brightness for visited pixels.
        }

        // Turn on the animation pixel with reduced brightness.
        const Point& animPoint = animationPath[animationIndex];
        img.setPixelValue(animPoint.x, animPoint.y, 15); // Reduced brightness (e.g., 20 out of 255).
        animationIndex = (animationIndex + 1) % animationPathLength;

        // Update the pixel at the user's position.
        img.setPixelValue(cursor.x, cursor.y, 255); // reduced brightness (e.g., 1 out of 255).

        // Update the buffer to the screen.
        display.image.paste(img, 0, 0, 0);

        // Take a snapshot of the current accelerometer data.
        int x = accelerometer.getX();
        int y = accelerometer.getY();

        // Determine the position of the user-controlled pixel on the screen.
        cursor.x = (x < -PIXEL2_THRESHOLD) ? 0 :
                   (x < -PIXEL1_THRESHOLD) ? 1 :
                   (x > PIXEL2_THRESHOLD) ? 4 :
                   (x > PIXEL1_THRESHOLD) ? 3 : 2;

        cursor.y = (y < -PIXEL2_THRESHOLD) ? 0 :
                   (y < -PIXEL1_THRESHOLD) ? 1 :
                   (y > PIXEL2_THRESHOLD) ? 4 :
                   (y > PIXEL1_THRESHOLD) ? 3 : 2;

        // Test if we need to update the state at the user's position.
        for (int i = 0; i < PERIMETER_POINTS; i++)
        {
            if (cursor.x == perimeter[i].x && cursor.y == perimeter[i].y && !visited[i])
            {
                // Record the sample data for later processing.
                data[samples] = compass.getSample(RAW);

                // Mark this pixel as visited.
                visited[i] = 1;
                samples++;
                break; // No need to check further once a match is found.
            }
        }

        target_wait(TIME_STEP);
    }

    CompassCalibration cal = calibrate(data, samples); 
    compass.setCalibration(cal);

    if(this->storage)
        this->storage->put("compassCal", (uint8_t *) &cal, sizeof(CompassCalibration));

    // Show a smiley to indicate that we're done, and continue on with the user program.
    display.clear();
    display.printAsync(smiley, 0, 0, 0, 1500);
    target_wait(1000);
    display.clear();

    // Retore the display brightness to the level it was at before this function was called.
    display.setBrightness(displayBrightness);
}
