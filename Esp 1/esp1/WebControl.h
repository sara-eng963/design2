#pragma once

#include <Arduino.h>

namespace app {

using CommandExecutor = bool (*)(const String&, String&);
using StatusJsonProvider = String (*)();

void configureWebControl(CommandExecutor executor, StatusJsonProvider statusProvider);
void beginWebControl();
void pollWebControl();

}  // namespace app