#pragma once

#include <stdio.h>
#include <iostream>

namespace legs::shell {

bool init();

bool runProgram(std::istream& sourceStream = std::cin);
bool runProgrram(FILE* sourceFile = stdin);


} // namespace legs::shell