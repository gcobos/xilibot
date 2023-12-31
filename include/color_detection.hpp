/* This file is taken from MyOwnBricks project.
 * MyOwnBricks is a library for the emulation of LEGO PoweredUp sensors on microcontrollers
 * Copyright (C) 2021-2022 Ysard - <ysard@users.noreply.github.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @brief Discretize colors and return uint8_t color code.
 *    Available colors: COLOR_NONE, COLOR_BLACK, COLOR_BLUE,
 *    COLOR_GREEN, COLOR_RED, COLOR_WHITE.
 *
 *    Generally speaking, stable measuring conditions are required, i.e.,
 *    a stable measuring distance not exceeding 4 cm, no interfering light
 *    reaching the side of the sensor. Think about matt black sensor shroud.
 *
 *    Metrics:
 *      - COLOR_DETECTION_BASIC_RGB: Simple comparison between channels;
 *          Very fast but is likely to produce errors.
 *      - COLOR_DETECTION_MANHATTAN: Sum of absolute values of distances.
 *          Quite heavy, but quite accurate if the reference values have been
 *          measured seriously and if the measurement environment is controlled
 *          (reproducible). Distance between the sensor and the object should be
 *          the same as during learning.
 *          https://fr.wikipedia.org/wiki/Distance_de_Manhattan
 *     - COLOR_DETECTION_CANBERRA: A weighted version of Manhattan distance;
 *          Very heavy but brings a higher accuracy and more tolerance/stability to variations
 *          in the measurement environment.
 *          Note: The manipulation of decimal numbers should be avoided
 *          on microcontrollers... Does it worth it? Probably not.
 *          https://en.wikipedia.org/wiki/Canberra_distance
 */

// Colors (detected & LED (except NONE for this last one)) expected values
#define COLOR_NONE      0xFF
#define COLOR_BLACK     0
#define COLOR_PINK      1
#define COLOR_PURPLE    2
#define COLOR_BLUE      3
#define COLOR_LIGHTBLUE 4
#define COLOR_CYAN      5
#define COLOR_GREEN     6
#define COLOR_YELLOW    7
#define COLOR_ORANGE    8
#define COLOR_RED       9
#define COLOR_WHITE     10


#ifdef COLOR_DETECTION_BASIC_RGB
uint8_t detectColor(const uint16_t &red, const uint16_t &green, const uint16_t &blue) {
    if ((red > green) && (red > blue)) {
        return COLOR_RED;
    } else if ((green > red) && (green > blue)) {
        return COLOR_GREEN;
    } else if ((blue > red) && (blue > green)) {
        return COLOR_BLUE;
    }
    return COLOR_NONE;
}
#endif

#if (defined(COLOR_DETECTION_MANHATTAN) || defined(COLOR_DETECTION_CANBERRA))
// *_1: measures at 1 cm
// *_3: measures at 3 cms
const uint16_t SAMPLES[][3] = {
    { 297,  83,  56 }, // RED_1
    {  43,  20,  17 }, // RED_3
    {  35, 142, 193 }, // BLUE_1
    {  35,  94, 116 }, // BLUE_3
    {  86, 257, 257 }, // CYAN_1
    {  36,  98,  97 }, // CYAN_3
    { 120, 141,  46 }, // YELLOW_1
    {  72,  73,  30 }, // YELLOW_3
    { 338, 373, 120 }, // YELLOW_PLQ_1
    { 159, 267, 201 }, // WHITE_1
    {  87, 126, 102 }, // WHITE_3
    {  89, 322, 163 }, // GREEN_1
    {  58, 106,  68 }, // GREEN_3
    { 103, 189,  57 }, // GREEN_LIGHT_1
    {  51,  77,  33 }, // GREEN_LIGHT_3
    {  26,  34,  28 }  // BLACK_1
};

const uint8_t SAMPLES_MAP[] = {
    COLOR_RED,    COLOR_RED,
    COLOR_BLUE,   COLOR_BLUE,
    COLOR_BLUE,   COLOR_BLUE,
    COLOR_YELLOW, COLOR_YELLOW,COLOR_YELLOW,
    COLOR_WHITE,  COLOR_WHITE,
    COLOR_GREEN,  COLOR_GREEN,
    COLOR_GREEN,  COLOR_GREEN,
    COLOR_BLACK
};

// Number of samples
const uint8_t samplesCount = sizeof(SAMPLES) / sizeof(SAMPLES[0]);

uint8_t detectColor(const uint16_t &red, const uint16_t &green, const uint16_t &blue) {
#ifdef COLOR_DETECTION_MANHATTAN
    uint16_t minDist = 10000;
    uint16_t expDist;
#else
    float minDist = 3;
    float expDist;
#endif
    uint8_t bestSampleIndex = 0;

    for (uint8_t i = 0; i < samplesCount; i++) {
#ifdef COLOR_DETECTION_MANHATTAN
        expDist = abs(static_cast<int16_t>(red - SAMPLES[i][0]))
                  + abs(static_cast<int16_t>(green - SAMPLES[i][1]))
                  + abs(static_cast<int16_t>(blue - SAMPLES[i][2]));
#else
        // Yeah it's ugly but abs() of Arduino is a macro different from the stl implementation
        // moreover the parameter must be explicitly signed.
        // The numerator or denominator must be a float.
        // https://www.best-microcontroller-projects.com/arduino-absolute-value.html
        // https://github.com/arduino/reference-en/issues/362
        expDist = (abs(static_cast<int16_t>(red - SAMPLES[i][0])) / static_cast<float>(red + SAMPLES[i][0]))
                  + (abs(static_cast<int16_t>(green - SAMPLES[i][1])) / static_cast<float>(green + SAMPLES[i][1]))
                  + (abs(static_cast<int16_t>(blue - SAMPLES[i][2])) / static_cast<float>(blue + SAMPLES[i][2]));
#endif
        if (expDist < minDist) {
            bestSampleIndex = i;
            minDist         = expDist;
        }
    }

    // Arbitrary threshold to avoid erroneous identifications
#ifdef COLOR_DETECTION_MANHATTAN
    if (minDist > 100) {
#else
    if (minDist > 1.9) { // Red color is quite difficult to identify even with this high threashold
#endif
        // Matching is not acceptable
        return COLOR_NONE;
    }
    // Get color value expected by the hub
    return SAMPLES_MAP[bestSampleIndex];
}


#endif
