// S-expression reader for Specctra DSN files.
#pragma once
#include <string>
#include <vector>

struct Sx {
    std::string atom;          // the token (quotes stripped) when this is an atom
    std::string raw;           // the token as written (with its quotes) when this is an atom
    bool list = false;
    std::vector<Sx> kids;

    const std::string& head() const;                  // first atom of a list ("" if none)
    const Sx* find(const std::string& key) const;     // first child list whose head is `key`
    std::vector<const Sx*> all(const std::string& key) const;
    double num(size_t i) const;                       // kids[i] as a number
    const std::string& str(size_t i) const;           // kids[i]'s atom
};

Sx parse_sexpr(const std::string& text);
