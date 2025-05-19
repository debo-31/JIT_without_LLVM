#ifndef COMPILER_COMPONENTS_H
#define COMPILER_COMPONENTS_H

#include <string>
#include <vector>
#include <iostream> // Using iostream for basic output, acknowledging this is an API

// Enum for token types
enum TokenType {
    TOKEN_NUMBER,
    TOKEN_PLUS,
    TOKEN_MINUS,
    TOKEN_MULTIPLY,
    TOKEN_DIVIDE,
    TOKEN_LPAREN,
    TOKEN_RPAREN,
    TOKEN_EOF, // End of File
    TOKEN_ERROR
};

// Struct to represent a token
struct Token {
    TokenType type;
    double value;  // Changed from int to double for floating-point support
    std::string lexeme;
    
    // Constructor
    Token(TokenType t, double v = 0.0, const std::string& l = "")
        : type(t), value(v), lexeme(l) {}
};

// Enum for AST Node types
enum ASTNodeType {
    AST_NUMBER,
    AST_BINARY_OP
};

// Struct for AST Nodes
struct ASTNode {
    ASTNodeType type;
    TokenType op_type;  // For binary operations
    double value;       // For numbers
    ASTNode* left;      // For binary operations
    ASTNode* right;     // For binary operations
    
    // Constructor for number nodes
    ASTNode(double val) : type(AST_NUMBER), value(val), left(nullptr), right(nullptr) {}
    
    // Constructor for binary operation nodes
    ASTNode(TokenType op, ASTNode* l, ASTNode* r) 
        : type(AST_BINARY_OP), op_type(op), value(0.0), left(l), right(r) {}
    
    ~ASTNode() {
        delete left;
        delete right;
    }
};

// Function declarations
std::vector<Token> lex(const std::string& source_code);
ASTNode* parse(const std::vector<Token>& tokens);
void print_ast(const ASTNode* node, int indent = 0);

#endif // COMPILER_COMPONENTS_H 