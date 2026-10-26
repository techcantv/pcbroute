#include "sexpr.h"

#include <cstdlib>
#include <stdexcept>

static const std::string EMPTY;

const std::string& Sx::head() const { return list && !kids.empty() && !kids[0].list ? kids[0].atom : EMPTY; }

const Sx* Sx::find(const std::string& key) const {
    for (auto& k : kids)
        if (k.list && k.head() == key) return &k;
    return nullptr;
}

std::vector<const Sx*> Sx::all(const std::string& key) const {
    std::vector<const Sx*> r;
    for (auto& k : kids)
        if (k.list && k.head() == key) r.push_back(&k);
    return r;
}

double Sx::num(size_t i) const {
    if (i >= kids.size() || kids[i].list) throw std::runtime_error("DSN: expected a number in (" + head() + ")");
    return std::strtod(kids[i].atom.c_str(), nullptr);
}

const std::string& Sx::str(size_t i) const {
    if (i >= kids.size() || kids[i].list) throw std::runtime_error("DSN: expected an atom in (" + head() + ")");
    return kids[i].atom;
}

Sx parse_sexpr(const std::string& t) {
    // Iterative parser: a stack of open lists. `(string_quote ")` names the quote character with a bare quote,
    // so the token after string_quote is read as one character.
    std::vector<Sx> stack(1);
    stack[0].list = true;
    size_t i = 0, n = t.size();
    char quote = '"';
    bool quote_next = false;
    while (i < n) {
        char c = t[i];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') { ++i; continue; }
        if (quote_next) {
            Sx a; a.atom = a.raw = std::string(1, c); quote = c;
            stack.back().kids.push_back(std::move(a));
            quote_next = false; ++i; continue;
        }
        if (c == '(') { stack.emplace_back(); stack.back().list = true; ++i; continue; }
        if (c == ')') {
            if (stack.size() < 2) throw std::runtime_error("DSN: unbalanced ')'");
            Sx done = std::move(stack.back());
            stack.pop_back();
            stack.back().kids.push_back(std::move(done));
            ++i; continue;
        }
        Sx a;
        if (c == quote) {
            size_t e = t.find(quote, i + 1);
            if (e == std::string::npos) throw std::runtime_error("DSN: unterminated string");
            a.atom = t.substr(i + 1, e - i - 1);
            a.raw = t.substr(i, e - i + 1);
            i = e + 1;
        } else {
            size_t s = i;
            while (i < n && t[i] != ' ' && t[i] != '\t' && t[i] != '\n' && t[i] != '\r' && t[i] != '(' && t[i] != ')')
                ++i;
            a.atom = a.raw = t.substr(s, i - s);
        }
        if (a.atom == "string_quote" && stack.back().kids.empty()) quote_next = true;
        stack.back().kids.push_back(std::move(a));
    }
    if (stack.size() != 1 || stack[0].kids.empty()) throw std::runtime_error("DSN: unbalanced '('");
    return std::move(stack[0].kids[0]);
}
