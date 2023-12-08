class array{
    private:
        int len;
        int *arr;
        int index = 0;
    

    public:
        array(int maxlen){
            this->len = maxlen;
            this->arr = new int[len];
        }
        void push_back(int element){
            arr[this->index] = element;
            this->index++;
        }
    
};