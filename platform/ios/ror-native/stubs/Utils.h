#pragma once

// RigDef_Parser only needs utf8cpp's validation helpers from the desktop
// Utils include. Pull those directly so the native parser target does not
// inherit MyGUI just to sanitize .truck text.
#include "utf8/checked.h"
#include "utf8/unchecked.h"
