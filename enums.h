//
// Created by Ben Eisner on 5/28/16.
//

#ifndef INC_3D_PROCESSING_ENUMS_H
#define INC_3D_PROCESSING_ENUMS_H

// Viewer.cpp
enum EditType { RotationMode, TranslationMode };
enum ColorMode { None, PrincipalCurvature, MeanCurvature, GaussianCurvature, SpectralWeight };
enum DisplayMode { Smooth, WireFrame, PointCloud };

// Button.cpp
enum PressState { NotPressed, Pressed };
enum HoverState { NoHover, Hover };

#endif //INC_3D_PROCESSING_ENUMS_H
