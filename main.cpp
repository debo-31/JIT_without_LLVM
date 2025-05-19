#include "jit_compiler.h"
#include <iostream>
#include <string>

int main() {
    Compiler compiler;
    std::string input;
    std::cout << "Enter an arithmetic expression: ";
    std::getline(std::cin, input);
    double result = compiler.compileAndExecute(input);
    std::cout << result << std::endl;
    return 0;
} 