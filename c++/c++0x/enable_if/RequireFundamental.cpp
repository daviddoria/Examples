template <class T>
struct X
{
typename enable_if<is_fundamental<T>::value>::type  f(T junk = T()) {}
};
int main() {X<int> x;}

struct Y {};
template <class T>
struct X
{
typename enable_if<is_fundamental<T>::value>::type  f(T junk = T()) {}
};

int main() {X<Y> x;}