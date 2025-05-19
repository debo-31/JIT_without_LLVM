#include "jit_compiler.h"
#include <iostream>
#include <stack>
#include <vector>
#include <iomanip>

// Virtual Machine execution
double VirtualMachine::execute() {
    std::cout << "\n4. Virtual Machine:" << std::endl;
    std::cout << "   Stack operations:" << std::endl;
    std::cout << "   []";
    
    for (const auto& instr : code) {
        switch (instr.type) {
            case Instruction::PUSH:
                stack.push(instr.value);
                std::cout << " → [" << std::fixed << std::setprecision(0) << instr.value << "]";
                break;
                
            case Instruction::ADD: {
                if (stack.size() < 2) {
                    throw std::runtime_error("Stack underflow during addition");
                }
                double b = stack.top(); stack.pop();
                double a = stack.top(); stack.pop();
                stack.push(a + b);
                std::cout << " → [" << std::fixed << std::setprecision(0) << (a + b) << "]";
                break;
            }
                
            case Instruction::SUB: {
                if (stack.size() < 2) {
                    throw std::runtime_error("Stack underflow during subtraction");
                }
                double b = stack.top(); stack.pop();
                double a = stack.top(); stack.pop();
                stack.push(a - b);
                std::cout << " → [" << std::fixed << std::setprecision(0) << (a - b) << "]";
                break;
            }
                
            case Instruction::MUL: {
                if (stack.size() < 2) {
                    throw std::runtime_error("Stack underflow during multiplication");
                }
                double b = stack.top(); stack.pop();
                double a = stack.top(); stack.pop();
                stack.push(a * b);
                std::cout << " → [" << std::fixed << std::setprecision(0) << (a * b) << "]";
                break;
            }
                
            case Instruction::DIV: {
                if (stack.size() < 2) {
                    throw std::runtime_error("Stack underflow during division");
                }
                double b = stack.top(); stack.pop();
                double a = stack.top(); stack.pop();
                if (b == 0.0) {
                    throw std::runtime_error("Division by zero");
                }
                stack.push(a / b);
                std::cout << " → [" << std::fixed << std::setprecision(0) << (a / b) << "]";
                break;
            }
                
            case Instruction::RET:
                if (stack.empty()) {
                    throw std::runtime_error("Stack empty during return");
                }
                std::cout << " → returns " << std::fixed << std::setprecision(0) << stack.top() << std::endl;
                return stack.top();
        }
    }
    
    throw std::runtime_error("No return instruction found");
}

// Compiler implementation
double Compiler::compileAndExecute(const std::string& source) {
    // Clear previous state
    vm.clear();
    irBuilder.clear();
    
    // Lex and parse
    auto tokens = lex(source);
    
    std::cout << "\n2. Parser:" << std::endl;
    std::cout << "   Creates AST:" << std::endl;
    ASTNode* ast = parse(tokens);
    if (!ast) {
        return -1;
    }
    print_ast(ast, 0);
    std::cout << std::endl;
    
    // Generate IR
    std::cout << "\n3. IR Generation:" << std::endl;
    std::cout << "   Generates IR instructions:" << std::endl;
    int resultReg = irGen.generateIR(ast);
    irBuilder.addInstruction(IRInstruction(IROp::RET, 0.0, resultReg));
    
    // Print IR
    for (const auto& instr : irBuilder.getInstructions()) {
        std::cout << "   ";
        switch (instr.op) {
            case IROp::CONST:
                std::cout << "r" << instr.dest << " = CONST " << std::fixed << std::setprecision(0) << instr.value;
                break;
            case IROp::ADD:
                std::cout << "r" << instr.dest << " = ADD r" << instr.src1 << " r" << instr.src2;
                break;
            case IROp::SUB:
                std::cout << "r" << instr.dest << " = SUB r" << instr.src1 << " r" << instr.src2;
                break;
            case IROp::MUL:
                std::cout << "r" << instr.dest << " = MUL r" << instr.src1 << " r" << instr.src2;
                break;
            case IROp::DIV:
                std::cout << "r" << instr.dest << " = DIV r" << instr.src1 << " r" << instr.src2;
                break;
            case IROp::RET:
                std::cout << "RET r" << instr.dest;
                break;
        }
        std::cout << std::endl;
    }
    
    // Generate VM code from IR
    codeGen.generateCode(irBuilder.getInstructions());
    
    // Execute
    double result = vm.execute();
    
    // Generate assembly code
    asmGen.generateCode(irBuilder.getInstructions());
    asmGen.printCode();
    
    // Cleanup
    delete ast;
    
    return result;
} 