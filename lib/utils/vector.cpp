template <typename T>
class vector
{
private:
    T *arr;
    int size;
    int capacity;

    void resize(int newCapacity)
    {
        T *newArr = new T[newCapacity];
        for (int i = 0; i < size; i++)
        {
            newArr[i] = arr[i];
        }
        delete[] arr;
        arr = newArr;
        capacity = newCapacity;
    }

public:
    vector() : arr(new T[10]), size(0), capacity(10) {}

    ~vector()
    {
        delete[] arr;
    }

    void push_back(T element)
    {
        if (size >= capacity)
        {
            resize(capacity * 2);
        }
        arr[size] = element;
        size++;
    }

    T get(int index)
    {
        return arr[index];
    }

    T pop()
    {
        if (size != 0) size--;
        return arr[size];
    }

    int length() const
    {
        return size;
    }
    
};
