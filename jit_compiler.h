#ifndef JIT_COMPILER_H
#define JIT_COMPILER_H

#include "compiler_components.h"
#include <vector>
#include <stack>
#include <string>
#include <memory>
#include <unordered_map>

// IR Instruction types
enum class IROp {
    CONST,      // Load constant
    ADD,        // Addition
    SUB,        // Subtraction
    MUL,        // Multiplication
    DIV,        // Division
    RET         // Return
};

// IR Instruction
struct IRInstruction {
    IROp op;
    double value;  // Used for CONST instruction
    int dest;      // Destination register
    int src1;      // First source register
    int src2;      // Second source register

    IRInstruction(IROp o, double v = 0.0, int d = -1, int s1 = -1, int s2 = -1)
        : op(o), value(v), dest(d), src1(s1), src2(s2) {}
};

// IR Builder
class IRBuilder {
private:
    std::vector<IRInstruction> instructions;
    int nextReg;

public:
    IRBuilder() : nextReg(0) {}

    int allocateRegister() {
        return nextReg++;
    }

    void addInstruction(const IRInstruction& instr) {
        instructions.push_back(instr);
    }

    const std::vector<IRInstruction>& getInstructions() const {
        return instructions;
    }

    void clear() {
        instructions.clear();
        nextReg = 0;
    }
};

// x86-64 Register names
enum class X86Register {
    // General purpose registers
    RAX, RBX, RCX, RDX, RSI, RDI, RBP, RSP,
    R8, R9, R10, R11, R12, R13, R14, R15,
    // Floating point registers
    XMM0, XMM1, XMM2, XMM3, XMM4, XMM5, XMM6, XMM7,
    XMM8, XMM9, XMM10, XMM11, XMM12, XMM13, XMM14, XMM15
};

// Register allocation strategy
enum class RegisterType {
    INTEGER,
    FLOAT
};

class RegisterAllocator {
private:
    std::vector<X86Register> intRegs;
    std::vector<X86Register> floatRegs;
    std::unordered_map<int, X86Register> regMap;
    std::unordered_map<int, RegisterType> regTypes;
    size_t nextIntReg;
    size_t nextFloatReg;

public:
    RegisterAllocator() : nextIntReg(0), nextFloatReg(0) {
        // Initialize integer registers
        intRegs = {
            X86Register::RAX, X86Register::RBX, X86Register::RCX, X86Register::RDX,
            X86Register::RSI, X86Register::RDI, X86Register::R8, X86Register::R9,
            X86Register::R10, X86Register::R11, X86Register::R12, X86Register::R13,
            X86Register::R14, X86Register::R15
        };
        
        // Initialize floating-point registers
        floatRegs = {
            X86Register::XMM0, X86Register::XMM1, X86Register::XMM2, X86Register::XMM3,
            X86Register::XMM4, X86Register::XMM5, X86Register::XMM6, X86Register::XMM7,
            X86Register::XMM8, X86Register::XMM9, X86Register::XMM10, X86Register::XMM11,
            X86Register::XMM12, X86Register::XMM13, X86Register::XMM14, X86Register::XMM15
        };
    }

    X86Register allocateRegister(int regId, RegisterType type) {
        X86Register reg;
        if (type == RegisterType::INTEGER) {
            if (nextIntReg >= intRegs.size()) {
                throw std::runtime_error("No more integer registers available");
            }
            reg = intRegs[nextIntReg++];
        } else {
            if (nextFloatReg >= floatRegs.size()) {
                throw std::runtime_error("No more floating-point registers available");
            }
            reg = floatRegs[nextFloatReg++];
        }
        regMap[regId] = reg;
        regTypes[regId] = type;
        return reg;
    }

    X86Register getRegister(int regId) const {
        auto it = regMap.find(regId);
        if (it == regMap.end()) {
            throw std::runtime_error("Register not allocated");
        }
        return it->second;
    }

    RegisterType getRegisterType(int regId) const {
        auto it = regTypes.find(regId);
        if (it == regTypes.end()) {
            throw std::runtime_error("Register type not found");
        }
        return it->second;
    }

    void clear() {
        regMap.clear();
        regTypes.clear();
        nextIntReg = 0;
        nextFloatReg = 0;
    }
};

// System call numbers for x86-64 Linux
enum class SysCall {
    READ = 0,
    WRITE = 1,
    EXIT = 60
};

// Error codes
enum class ErrorCode {
    SUCCESS = 0,
    DIVISION_BY_ZERO = 1,
    STACK_UNDERFLOW = 2,
    INVALID_OPERATION = 3,
    REGISTER_OVERFLOW = 4
};

// Assembly Code Generator
class AssemblyGenerator {
private:
    std::vector<std::string> code;
    RegisterAllocator regAlloc;
    bool isFloatOperation;

    std::string regToString(X86Register reg) {
        switch (reg) {
            case X86Register::RAX: return "rax";
            case X86Register::RBX: return "rbx";
            case X86Register::RCX: return "rcx";
            case X86Register::RDX: return "rdx";
            case X86Register::RSI: return "rsi";
            case X86Register::RDI: return "rdi";
            case X86Register::RBP: return "rbp";
            case X86Register::RSP: return "rsp";
            case X86Register::R8: return "r8";
            case X86Register::R9: return "r9";
            case X86Register::R10: return "r10";
            case X86Register::R11: return "r11";
            case X86Register::R12: return "r12";
            case X86Register::R13: return "r13";
            case X86Register::R14: return "r14";
            case X86Register::R15: return "r15";
            case X86Register::XMM0: return "xmm0";
            case X86Register::XMM1: return "xmm1";
            case X86Register::XMM2: return "xmm2";
            case X86Register::XMM3: return "xmm3";
            case X86Register::XMM4: return "xmm4";
            case X86Register::XMM5: return "xmm5";
            case X86Register::XMM6: return "xmm6";
            case X86Register::XMM7: return "xmm7";
            case X86Register::XMM8: return "xmm8";
            case X86Register::XMM9: return "xmm9";
            case X86Register::XMM10: return "xmm10";
            case X86Register::XMM11: return "xmm11";
            case X86Register::XMM12: return "xmm12";
            case X86Register::XMM13: return "xmm13";
            case X86Register::XMM14: return "xmm14";
            case X86Register::XMM15: return "xmm15";
            default: return "unknown";
        }
    }

    bool isFloatingPoint(double value) {
        return value != static_cast<int64_t>(value);
    }

public:
    AssemblyGenerator() : isFloatOperation(false) {}

    void generateCode(const std::vector<IRInstruction>& ir) {
        code.clear();
        regAlloc.clear();

        // Function prologue
        code.push_back("section .text");
        code.push_back("global _start");
        code.push_back("_start:");
        code.push_back("    push rbp");
        code.push_back("    mov rbp, rsp");
        code.push_back("    sub rsp, 16");  // Reserve stack space for floating-point operations

        // First pass: allocate registers for all dests, with correct type
        for (const auto& instr : ir) {
            switch (instr.op) {
                case IROp::CONST: {
                    bool isFloat = isFloatingPoint(instr.value);
                    regAlloc.allocateRegister(instr.dest, isFloat ? RegisterType::FLOAT : RegisterType::INTEGER);
                    break;
                }
                case IROp::ADD:
                case IROp::SUB:
                case IROp::MUL:
                case IROp::DIV: {
                    // Infer type from src1 (assume src1 always allocated before dest)
                    RegisterType type = regAlloc.getRegisterType(instr.src1);
                    regAlloc.allocateRegister(instr.dest, type);
                    break;
                }
                case IROp::RET:
                    // RET does not allocate a new register
                    break;
            }
        }

        // Second pass: generate code
        for (const auto& instr : ir) {
            switch (instr.op) {
                case IROp::CONST: {
                    bool isFloat = isFloatingPoint(instr.value);
                    X86Register reg = regAlloc.getRegister(instr.dest);
                    if (isFloat) {
                        code.push_back("    movsd xmm0, [rel float_const_" + std::to_string(instr.dest) + "]");
                        code.push_back("    movsd [rsp], xmm0");
                        code.push_back("    movsd " + regToString(reg) + ", [rsp]");
                    } else {
                        code.push_back("    mov " + regToString(reg) + ", " + 
                            std::to_string(static_cast<int64_t>(instr.value)));
                    }
                    break;
                }
                case IROp::ADD: {
                    X86Register dest = regAlloc.getRegister(instr.dest);
                    X86Register src1 = regAlloc.getRegister(instr.src1);
                    X86Register src2 = regAlloc.getRegister(instr.src2);
                    if (regAlloc.getRegisterType(instr.dest) == RegisterType::FLOAT) {
                        code.push_back("    movsd " + regToString(dest) + ", " + regToString(src1));
                        code.push_back("    addsd " + regToString(dest) + ", " + regToString(src2));
                    } else {
                        code.push_back("    mov " + regToString(dest) + ", " + regToString(src1));
                        code.push_back("    add " + regToString(dest) + ", " + regToString(src2));
                    }
                    break;
                }
                case IROp::SUB: {
                    X86Register dest = regAlloc.getRegister(instr.dest);
                    X86Register src1 = regAlloc.getRegister(instr.src1);
                    X86Register src2 = regAlloc.getRegister(instr.src2);
                    if (regAlloc.getRegisterType(instr.dest) == RegisterType::FLOAT) {
                        code.push_back("    movsd " + regToString(dest) + ", " + regToString(src1));
                        code.push_back("    subsd " + regToString(dest) + ", " + regToString(src2));
                    } else {
                        code.push_back("    mov " + regToString(dest) + ", " + regToString(src1));
                        code.push_back("    sub " + regToString(dest) + ", " + regToString(src2));
                    }
                    break;
                }
                case IROp::MUL: {
                    X86Register dest = regAlloc.getRegister(instr.dest);
                    X86Register src1 = regAlloc.getRegister(instr.src1);
                    X86Register src2 = regAlloc.getRegister(instr.src2);
                    if (regAlloc.getRegisterType(instr.dest) == RegisterType::FLOAT) {
                        code.push_back("    movsd " + regToString(dest) + ", " + regToString(src1));
                        code.push_back("    mulsd " + regToString(dest) + ", " + regToString(src2));
                    } else {
                        code.push_back("    mov rax, " + regToString(src1));
                        code.push_back("    mul " + regToString(src2));
                        code.push_back("    mov " + regToString(dest) + ", rax");
                    }
                    break;
                }
                case IROp::DIV: {
                    X86Register dest = regAlloc.getRegister(instr.dest);
                    X86Register src1 = regAlloc.getRegister(instr.src1);
                    X86Register src2 = regAlloc.getRegister(instr.src2);
                    if (regAlloc.getRegisterType(instr.dest) == RegisterType::FLOAT) {
                        code.push_back("    movsd " + regToString(dest) + ", " + regToString(src1));
                        code.push_back("    divsd " + regToString(dest) + ", " + regToString(src2));
                    } else {
                        code.push_back("    mov rax, " + regToString(src1));
                        code.push_back("    xor rdx, rdx");
                        code.push_back("    test " + regToString(src2) + ", " + regToString(src2));
                        code.push_back("    jz division_by_zero");
                        code.push_back("    div " + regToString(src2));
                        code.push_back("    mov " + regToString(dest) + ", rax");
                    }
                    break;
                }
                case IROp::RET: {
                    X86Register result = regAlloc.getRegister(instr.dest);
                    if (regAlloc.getRegisterType(instr.dest) == RegisterType::FLOAT) {
                        code.push_back("    movsd xmm0, " + regToString(result));
                    } else {
                        code.push_back("    mov rax, " + regToString(result));
                    }
                    code.push_back("    mov rsp, rbp");
                    code.push_back("    pop rbp");
                    code.push_back("    ret");
                    break;
                }
            }
        }
    }

    void printCode() const {
        std::cout << "\n5. Assembly Code:" << std::endl;
        for (const auto& line : code) {
            std::cout << "   " << line << std::endl;
        }
    }
};

// Virtual Machine instruction types
enum class Instruction {
    PUSH,   // Push value onto stack
    ADD,    // Add top two values on stack
    SUB,    // Subtract top two values on stack
    MUL,    // Multiply top two values on stack
    DIV,    // Divide top two values on stack
    RET     // Return top value from stack
};

// Virtual Machine instruction
struct VMInstruction {
    Instruction type;
    double value;  // Used for PUSH instruction

    VMInstruction(Instruction t, double v = 0.0) : type(t), value(v) {}
};

// Stack-based Virtual Machine
class VirtualMachine {
private:
    std::vector<VMInstruction> code;
    std::stack<double> stack;

public:
    VirtualMachine() = default;
    
    void addInstruction(const VMInstruction& instr) {
        code.push_back(instr);
    }
    
    void clear() {
        code.clear();
        while (!stack.empty()) stack.pop();
    }
    
    const std::vector<VMInstruction>& getCode() const {
        return code;
    }
    
    double execute();
};

// IR to VM Code Generator
class IRCodeGenerator {
private:
    VirtualMachine& vm;
    std::vector<double> registers;

public:
    IRCodeGenerator(VirtualMachine& virtualMachine) : vm(virtualMachine) {}

    void generateCode(const std::vector<IRInstruction>& ir) {
        registers.clear();
        for (const auto& instr : ir) {
            switch (instr.op) {
                case IROp::CONST:
                    vm.addInstruction(VMInstruction(Instruction::PUSH, instr.value));
                    break;
                case IROp::ADD:
                    vm.addInstruction(VMInstruction(Instruction::ADD));
                    break;
                case IROp::SUB:
                    vm.addInstruction(VMInstruction(Instruction::SUB));
                    break;
                case IROp::MUL:
                    vm.addInstruction(VMInstruction(Instruction::MUL));
                    break;
                case IROp::DIV:
                    vm.addInstruction(VMInstruction(Instruction::DIV));
                    break;
                case IROp::RET:
                    vm.addInstruction(VMInstruction(Instruction::RET));
                    break;
            }
        }
    }
};

// AST to IR Generator
class IRGenerator {
private:
    IRBuilder& builder;

public:
    IRGenerator(IRBuilder& irBuilder) : builder(irBuilder) {}

    int generateIR(ASTNode* node) {
        if (!node) return -1;

        if (node->type == AST_NUMBER) {
            int reg = builder.allocateRegister();
            builder.addInstruction(IRInstruction(IROp::CONST, node->value, reg));
            return reg;
        } else if (node->type == AST_BINARY_OP) {
            int leftReg = generateIR(node->left);
            int rightReg = generateIR(node->right);
            int resultReg = builder.allocateRegister();

            IROp op;
            switch (node->op_type) {
                case TOKEN_PLUS: op = IROp::ADD; break;
                case TOKEN_MINUS: op = IROp::SUB; break;
                case TOKEN_MULTIPLY: op = IROp::MUL; break;
                case TOKEN_DIVIDE: op = IROp::DIV; break;
                default: throw std::runtime_error("Unsupported operator");
            }

            builder.addInstruction(IRInstruction(op, 0.0, resultReg, leftReg, rightReg));
            return resultReg;
        }
        return -1;
    }
};

// Compiler class
class Compiler {
private:
    VirtualMachine vm;
    IRBuilder irBuilder;
    IRGenerator irGen;
    IRCodeGenerator codeGen;
    AssemblyGenerator asmGen;
    
public:
    Compiler() : irGen(irBuilder), codeGen(vm) {}
    
    double compileAndExecute(const std::string& source);
};

#endif // JIT_COMPILER_H 