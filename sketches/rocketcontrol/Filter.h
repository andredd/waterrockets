template <typename T>
class SimpleBuffer {
  protected:
    T* buffer;
    int pos;
    int size;
  public:

    SimpleBuffer(int size) {
      buffer = new T[size];
      this->size = size;
      reset();
    }

    ~SimpleBuffer() {
      delete[] buffer;
    }

    int addVal(T val) {
      int wrPos = pos;
      this->buffer[pos] = val;
      pos++;
      if ( pos >= size)
        pos = 0;

      return wrPos;

    }

    T getVal(int pos) {
      return this->buffer[pos];
    }

    int getSize() {
      return size;
    }

    void reset() {
      memset(buffer, 0, sizeof(buffer));
      pos = 0;
    }
};

template <typename R>
class RollMeanBuffer: public SimpleBuffer<R>
{
  private:
    double mean;

  public:
    RollMeanBuffer(int size) : SimpleBuffer<R>(size){
      mean = 0;
    }

    double getMean() {
      return mean;
    }

    void reset() {
      SimpleBuffer<R>::reset();
      mean = 0;
    }

    int addVal(R val)
    {
      int res = SimpleBuffer<R>::addVal(val);
      mean = (mean * (this->size - 1) + val) / this->size;
      return res;
    }
};
