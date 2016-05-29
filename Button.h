//
// Created by Ben Eisner on 5/25/16.
//

#ifndef INC_3D_PROCESSING_BUTTON_H
#define INC_3D_PROCESSING_BUTTON_H


#include <string>
#include "Mouse.h"


// ---------------------- Button interaction System -------------------------
// Based on code found at: https://nccastaff.bournemouth.ac.uk/jmacey/RobTheBloke/www/opengl_programming.html
// Modified for my use
typedef void(*ButtonCallback)();

//struct for button
class Button
{
public:
    int x_top_left;
    int y_top_left;
    int w;				// width
    int h;				// height
    const char* label;		// button text
    std::string slabel;
    ButtonCallback callbackFunction;
    Mouse *TheMouse;

    int state = 0;			// if 1: then pressed, else 0: not pressed
    int highlighted = 0;		// is mouse over the button

    //const unsigned char* culabel;		// button text


    Button(int x_top_left, int y_top_left, int w, int h, const char* label, std::string slabel, ButtonCallback callback, Mouse *TheMouse);
    bool ClickTest(int x, int y);
    void ButtonRelease(int x, int y);
    void ButtonPress(int x, int y);
    bool ButtonPassive(int x, int y);


};

#endif //INC_3D_PROCESSING_BUTTON_H
