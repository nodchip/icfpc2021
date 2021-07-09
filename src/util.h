#pragma once
#include <filesystem>
#include <optional>
#include <iostream>
#include <nlohmann/json.hpp>

std::filesystem::path default_data_path();
std::filesystem::path default_problem_path(int num);

bool update_meta(nlohmann::json& solution_json, const std::string& solver_name);

std::vector<std::string> split(std::string s, std::string delimiter);
std::string join(std::vector<std::string> tokens, std::string delimiter);
std::pair<std::string, std::string> split_first(std::string s, std::string delimiter);
bool starts_with(std::string_view s, std::string prefix);
std::string strip(std::string s);
std::optional<std::string> read_file(std::filesystem::path file_path);

/* const */
constexpr double PI = 3.141592653589793238462643;

/* io */
// output tuple
namespace aux {
    template<typename T, unsigned N, unsigned L>
    struct tp {
        static void print(std::ostream& os, const T& v) {
            os << std::get<N>(v) << ", ";
            tp<T, N + 1, L>::print(os, v);
        }
    };
    template<typename T, unsigned N> struct tp<T, N, N> {
        static void print(std::ostream& os, const T& v) {
            os << std::get<N>(v);
        }
    };
}
template<typename... Ts>
std::ostream& operator<<(std::ostream& os, const std::tuple<Ts...>& t) {
    os << '[';
    aux::tp<std::tuple<Ts...>, 0, sizeof...(Ts) - 1>::print(os, t);
    os << ']';
    return os;
}
// fwd decl
template <class T, class = typename T::iterator, std::enable_if_t<!std::is_same<T, std::string>::value, int> = 0>
std::ostream& operator<<(std::ostream& os, T const& a);
// output pair
template <class T, class S> std::ostream& operator<<(std::ostream& os, std::pair<T, S> const& p) {
    return os << '[' << p.first << ", " << p.second << ']';
}
// input pair
template <class T, class S> std::istream& operator>>(std::istream& is, std::pair<T, S>& p) {
    return is >> p.first >> p.second;
}
// output container
template <class T, class, std::enable_if_t<!std::is_same<T, std::string>::value, int>>
std::ostream& operator<<(std::ostream& os, T const& a) {
    bool f = true;
    os << '[';
    for (auto const& x : a) {
        os << (f ? "" : ", ") << x;
        f = false;
    }
    os << ']';
    return os;
}
// output array
template <class T, size_t N, std::enable_if_t<!std::is_same<T, char>::value, int> = 0>
std::ostream& operator<<(std::ostream& os, const T(&a)[N]) {
    bool f = true;
    os << '[';
    for (auto const& x : a) {
        os << (f ? "" : ", ") << x;
        f = false;
    }
    os << ']';
    return os;
}
// input container
template <class T, class = decltype(std::begin(std::declval<T&>())), class = typename std::enable_if<!std::is_same<T, std::string>::value>::type>
std::istream& operator>>(std::istream& is, T& a) {
    for (auto& x : a) is >> x;
    return is;
}

/* format */
template<typename... Ts>
std::string format(const std::string& f, Ts... t) {
    size_t l = std::snprintf(nullptr, 0, f.c_str(), t...);
    std::vector<char> b(l + 1);
    std::snprintf(&b[0], l + 1, f.c_str(), t...);
    return std::string(&b[0], &b[0] + l);
}

/* debug */
#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG

#define DEBUGOUT std::cerr

#define debug(...) \
do { \
    DEBUGOUT << "  "; \
    DEBUGOUT << #__VA_ARGS__ << " :[DEBUG - " << __LINE__ << ":" << __FUNCTION__ << "]" << std::endl; \
    DEBUGOUT << "    "; \
    debug_func(__VA_ARGS__); \
} while(0);

void debug_func();

template <class Head, class... Tail>
void debug_func(Head&& head, Tail&&... tail) {
    DEBUGOUT << head;
    if (sizeof...(Tail) == 0) {
        DEBUGOUT << " ";
    }
    else {
        DEBUGOUT << ", ";
    }
    debug_func(std::move(tail)...);
}

#else
#define debug(...) void(0);
#endif

// vim:ts=2 sw=2 sts=2 et ci
