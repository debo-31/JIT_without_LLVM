#include "compiler_components.h"
#include <cctype> // For isspace and isdigit, acknowledging this is an API
#include <stdexcept>
#include <functional>
#include <iomanip>

// Very basic lexer function
std::vector<Token> lex(const std::string& input) {
    std::cout << "\n1. Lexer:" << std::endl;
    std::cout << "   Input: \"" << input << "\"" << std::endl;
    std::cout << "   Output: [";
    
    std::vector<Token> tokens;
    size_t current = 0;
    bool first_token = true;
    
    while (current < input.length()) {
        char c = input[current];
        
        // Skip whitespace
        if (isspace(c)) {
            current++;
            continue;
        }
        
        // Handle numbers (including floating-point)
        if (isdigit(c) || c == '.') {
            std::string num_str;
            bool has_decimal = false;
            
            while (current < input.length() && (isdigit(input[current]) || input[current] == '.')) {
                if (input[current] == '.') {
                    if (has_decimal) {
                        throw std::runtime_error("Invalid number format: multiple decimal points");
                    }
                    has_decimal = true;
                }
                num_str += input[current];
                current++;
            }
            
            double value = std::stod(num_str);
            tokens.push_back(Token(TOKEN_NUMBER, value, num_str));
            if (!first_token) std::cout << ", ";
            std::cout << "TOKEN_NUMBER(" << std::fixed << std::setprecision(0) << value << ")";
            first_token = false;
            continue;
        }
        
        // Handle operators
        switch (c) {
            case '+':
                tokens.push_back(Token(TOKEN_PLUS, 0.0, "+"));
                if (!first_token) std::cout << ", ";
                std::cout << "TOKEN_PLUS";
                break;
            case '-':
                tokens.push_back(Token(TOKEN_MINUS, 0.0, "-"));
                if (!first_token) std::cout << ", ";
                std::cout << "TOKEN_MINUS";
                break;
            case '*':
                tokens.push_back(Token(TOKEN_MULTIPLY, 0.0, "*"));
                if (!first_token) std::cout << ", ";
                std::cout << "TOKEN_MULTIPLY";
                break;
            case '/':
                tokens.push_back(Token(TOKEN_DIVIDE, 0.0, "/"));
                if (!first_token) std::cout << ", ";
                std::cout << "TOKEN_DIVIDE";
                break;
            case '(':
                tokens.push_back(Token(TOKEN_LPAREN, 0.0, "("));
                if (!first_token) std::cout << ", ";
                std::cout << "TOKEN_LPAREN";
                break;
            case ')':
                tokens.push_back(Token(TOKEN_RPAREN, 0.0, ")"));
                if (!first_token) std::cout << ", ";
                std::cout << "TOKEN_RPAREN";
                break;
            default:
                throw std::runtime_error("Unexpected character: " + std::string(1, c));
        }
        first_token = false;
        current++;
    }
    
    tokens.push_back(Token(TOKEN_EOF, 0.0, ""));
    std::cout << "]" << std::endl;
    return tokens;
}

// Helper function to parse a primary expression (number)
static ASTNode* parse_primary(const std::vector<Token>& tokens, size_t& current_token_index) {
    if (current_token_index >= tokens.size()) {
        std::cerr << "Error: Unexpected end of input" << std::endl;
        return nullptr;
    }

    const Token& current_token = tokens[current_token_index];
    if (current_token.type == TOKEN_NUMBER) {
        current_token_index++; // Consume the number token
        return new ASTNode(current_token.value);
    } else {
        std::cerr << "Error: Expected number, found token type " << current_token.type 
                  << " at index " << current_token_index << std::endl;
        return nullptr;
    }
}

// Function to parse a simple expression (NUMBER + NUMBER + ...)
ASTNode* parse(const std::vector<Token>& tokens) {
    size_t current = 0;
    std::function<ASTNode*()> expression;
    std::function<ASTNode*()> term;
    std::function<ASTNode*()> factor;

    auto current_token = [&]() -> const Token& {
        return tokens[current];
    };
    auto match = [&](TokenType type) -> bool {
        if (current < tokens.size() && current_token().type == type) {
            current++;
            return true;
        }
        return false;
    };
    factor = [&]() -> ASTNode* {
        if (current < tokens.size() && tokens[current].type == TOKEN_NUMBER) {
            double val = tokens[current].value;
            current++;
            return new ASTNode(val);
        }
        if (match(TOKEN_LPAREN)) {
            ASTNode* expr = expression();
            if (!match(TOKEN_RPAREN)) {
                throw std::runtime_error("Expected ')'");
            }
            return expr;
        }
        throw std::runtime_error("Expected number or '('");
    };
    term = [&]() -> ASTNode* {
        ASTNode* left = factor();
        while (current < tokens.size()) {
            if (match(TOKEN_MULTIPLY)) {
                left = new ASTNode(TOKEN_MULTIPLY, left, factor());
            } else if (match(TOKEN_DIVIDE)) {
                left = new ASTNode(TOKEN_DIVIDE, left, factor());
            } else {
                break;
            }
        }
        return left;
    };
    expression = [&]() -> ASTNode* {
        ASTNode* left = term();
        while (current < tokens.size()) {
            if (match(TOKEN_PLUS)) {
                left = new ASTNode(TOKEN_PLUS, left, term());
            } else if (match(TOKEN_MINUS)) {
                left = new ASTNode(TOKEN_MINUS, left, term());
            } else {
                break;
            }
        }
        return left;
    };
    return expression();
}

// Helper function to print the AST (for verification)
void print_ast(const ASTNode* node, int indent) {
    if (!node) return;

    if (node->type == AST_NUMBER) {
        std::cout << std::fixed << std::setprecision(0) << node->value;
    } else if (node->type == AST_BINARY_OP) {
        std::cout << std::string(indent * 2, ' ');
        switch (node->op_type) {
            case TOKEN_PLUS: std::cout << "+"; break;
            case TOKEN_MINUS: std::cout << "-"; break;
            case TOKEN_MULTIPLY: std::cout << "*"; break;
            case TOKEN_DIVIDE: std::cout << "/"; break;
            default: std::cout << "?"; break;
        }
        std::cout << std::endl;
        std::cout << std::string(indent * 2, ' ') << "/ \\" << std::endl;
        std::cout << std::string(indent * 2, ' ');
        print_ast(node->left, indent + 1);
        std::cout << " ";
        print_ast(node->right, indent + 1);
    }
} 