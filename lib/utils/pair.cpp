template <typename T1, typename T2>
class pair {
public:
    T1 first;
    T2 second;

    // Constructor
    pair(T1 f, T2 s) : first(f), second(s) {}

    // Default constructor
    pair() : first(T1()), second(T2()) {}
};
