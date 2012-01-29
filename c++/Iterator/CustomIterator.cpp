#include <iterator>

template <typename T>
struct PointedType;

template <typename T>
struct PointedType<T*>
{
  typedef T value_type;
};

using namespace std;

template <typename InputIterator>
struct DerefIterator
{
  typedef input_iterator_tag iterator_category;
  typedef typename PointedType<
            typename iterator_traits<InputIterator>::value_type>::value_type
            value_type;
  typedef typename iterator_traits<InputIterator>::difference_type
            difference_type;
  typedef value_type* pointer;
  typedef value_type& reference;

  public:
    explicit DerefIterator(const InputIterator& ii)
      : it(ii) {}

    // Returns the object pointed by the object referenced by it
    reference operator*()  const { return **it; }
    pointer   operator->() const { return *it; }

    DerefIterator& operator++()
    {
        ++it;
        return *this;
    }

    DerefIterator operator++(int)
    {
        DerefIterator tmp = *this;
        ++it;
        return tmp;
    }

    bool equals(const DerefIterator<InputIterator> & di) const
    {
        return di.it == it;
    }

  private:
    InputIterator it;
};

// Equality functions

template <typename InputIterator>
inline bool operator==(const DerefIterator<InputIterator>& di1,
                       const DerefIterator<InputIterator>& di2)
{
  return di1.equals(di2);
}

template <typename InputIterator>
inline bool operator!=(const DerefIterator<InputIterator>& di1,
                       const DerefIterator<InputIterator>& di2)
{
  return ! (di1 == di2);
}

//Helper function

template <typename InputIterator>
DerefIterator<InputIterator> deref_iterator(const InputIterator& ii)
{
  return DerefIterator<InputIterator>(ii);
}

int main()
{
  return 0;
};